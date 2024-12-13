from enum import Enum
from typing import Union

import jsonschema
import rclpy
import requests
from jsonschema.exceptions import ValidationError
from rclpy.node import Node
from crc import Calculator, Crc8

from minipock_custom_msgs.srv import (
    GetChunk,
    GetChunk_Response,
    TrigUpdate,
    TrigUpdate_Response,
)

STORED_VERSION_SCHEMA = {
    "type": "object",
    "patternProperties": {
        "^v[0-9]+\\.[0-9]+\\.[0-9]+$": {
            "type": "object",
            "properties": {
                "size": {"type": "integer"},
                "content": {"type": "string"},
            },
            "required": ["size", "content"],
        }
    },
    "additionalProperties": False,
}


class TrigUpdateErrorCode(Enum):
    """
    ErrorCode is an enumeration that represents various error codes that can be encountered
    in the firmware updater process.
    """

    SUCCESS = 0
    PARAMETER_NOT_DECLARED = 1
    INVALID_METHOD = 2
    GITHUB_FETCH_ERROR = 3
    NO_BIN_FILE = 4
    BIN_DOWNLOAD_ERROR = 5
    SCHEMA_VALIDATION_FAILED = 6


class GetChunkErrorCode(Enum):
    """
    ErrorCode is an enumeration that represents various error codes that can be encountered
    in the firmware updater process.
    """

    SUCCESS = 0
    VERSION_NOT_FOUND = 1
    END_OF_FILE = 2


class FirmwareUpdater(Node):
    """
    A ROS2 node that handles firmware updates for the minipock device.
    """

    def __init__(self):
        super().__init__("firmware_updater")
        self.declare_parameter("namespace", "minipock")
        self.declare_parameter("method", "latest")
        namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.stored_version = {}

        self.github_tags: list = self.__get_github_tags()
        self.allowed_methods: list = ["latest"] + self.github_tags
        self.get_logger().info(f"Allowed methods: {self.allowed_methods}")

        self.firmware_update_service = self.create_service(
            TrigUpdate, f"{namespace}/firmware_update", self.firmware_update_callback
        )

        self.firmware_chunk_service = self.create_service(
            GetChunk, f"{namespace}/firmware_update/chunk", self.firmware_chunk_callback
        )
        self.calculator = Calculator(Crc8.CCITT, optimized=True)

    def __get_github_tags(self) -> list:
        url = "https://api.github.com/repos/catie-aq/minipock_zephyr-demo/releases"
        response = requests.get(url, timeout=10)
        if response.status_code == 200:
            releases = response.json()
            tags = [release["tag_name"] for release in releases]
            return tags
        else:
            self.get_logger().error(
                f"Failed to fetch tags from GitHub: {response.status_code}"
            )
            return []

    def __return_no_update(
        self, error_code: TrigUpdateErrorCode
    ) -> TrigUpdate_Response:
        response = TrigUpdate_Response()
        response.success = error_code.value
        response.new_version_available = False
        response.new_version = ""
        response.size = 0
        return response

    def __get_method(self) -> Union[str, TrigUpdate_Response]:
        if self.has_parameter("method"):
            method = self.get_parameter("method").get_parameter_value().string_value
        else:
            self.get_logger().error("Parameter 'method' is not declared.")
            return self.__return_no_update(TrigUpdateErrorCode.PARAMETER_NOT_DECLARED)
        if method not in self.allowed_methods:
            return self.__return_no_update(TrigUpdateErrorCode.INVALID_METHOD)
        return method

    def __download_and_store_firmware(self, version) -> TrigUpdateErrorCode:
        url = f"https://api.github.com/repos/catie-aq/minipock_zephyr-demo/releases/tags/{version}"
        download_response = requests.get(url, timeout=10)
        if download_response.status_code != 200:
            self.get_logger().error(
                f"Failed to fetch release details from GitHub: {download_response.status_code}"
            )
            return TrigUpdateErrorCode.GITHUB_FETCH_ERROR
        release = download_response.json()
        bin_file = next(
            (
                asset
                for asset in release.get("assets", [])
                if asset["name"].endswith(".bin")
            ),
            None,
        )
        if not bin_file:
            self.get_logger().error("No .bin file found in the release assets.")
            return TrigUpdateErrorCode.NO_BIN_FILE
        bin_url = bin_file["browser_download_url"]
        bin_response = requests.get(bin_url, timeout=10, stream=True)
        if bin_response.status_code != 200:
            self.get_logger().error(
                f"Failed to download .bin file: {bin_response.status_code}"
            )
            return TrigUpdateErrorCode.BIN_DOWNLOAD_ERROR
        content = bytearray()
        for chunk in bin_response.iter_content(chunk_size=8192):
            if chunk:
                content.extend(chunk)
        json_version = {
            "size": len(content),
            "content": content.hex(),
        }
        self.stored_version[version] = json_version
        try:
            jsonschema.validate(
                instance=self.stored_version, schema=STORED_VERSION_SCHEMA
            )
        except ValidationError as e:
            self.get_logger().error(
                f"Stored version schema validation failed: {e.message}"
            )
            return TrigUpdateErrorCode.SCHEMA_VALIDATION_FAILED
        return TrigUpdateErrorCode.SUCCESS

    def firmware_update_callback(self, request, response) -> TrigUpdate_Response:
        """
        Callback function to handle firmware update requests.

        Args:
            request: The request object containing the actual firmware version.
            response: The response object to be populated with the update information.

        Returns:
            TrigUpdate_Response: The response object indicating whether a new firmware version
                                is available, along with the new version details if applicable.

        The function performs the following steps:
        1. Retrieves the update method.
        2. Determines the target firmware version based on the update method.
        3. Validates the actual and target firmware versions.
        4. Compares the actual firmware version with the target version.
        5. Downloads and stores the new firmware if an update is available.
        6. Populates the response object with the update information.
        """
        method = self.__get_method()
        if isinstance(method, TrigUpdate_Response):
            return method

        if method == "latest":
            version = self.github_tags[0] if self.github_tags else None
        else:
            version = method

        if not version:
            return self.__return_no_update(TrigUpdateErrorCode.SUCCESS)

        parsed_actual_version = request.actual_version.replace(".", "")[1:]
        parsed_version = version.replace(".", "")[1:]

        if (
            len(parsed_actual_version) != 3
            or len(parsed_version) != 3
            or not parsed_actual_version.isdigit()
            or not parsed_version.isdigit()
            or parsed_actual_version > self.github_tags[0].replace(".", "")[1:]
        ):
            return self.__return_no_update(TrigUpdateErrorCode.INVALID_METHOD)

        if parsed_actual_version >= parsed_version:
            return self.__return_no_update(TrigUpdateErrorCode.SUCCESS)

        error_code: TrigUpdateErrorCode = self.__download_and_store_firmware(version)
        if error_code != TrigUpdateErrorCode.SUCCESS:
            return self.__return_no_update(error_code)
        response.new_version_available = True
        response.new_version = version
        response.size = self.stored_version[version]["size"]

        return response

    def __return_no_chunk(self, error_code: GetChunkErrorCode) -> GetChunk_Response:
        response = GetChunk_Response()
        response.success = error_code.value
        response.chunk_id = 0
        response.chunk_byte = ""
        response.chunk_checksum = 0
        return response

    def firmware_chunk_callback(self, request, response):
        if request.version not in self.stored_version:
            self.get_logger().error(f"Version {request.version} not found.")
            return self.__return_no_chunk(GetChunkErrorCode.VERSION_NOT_FOUND)
        version = self.stored_version[request.version]
        update_size = version["size"]
        chunk_id = request.chunk_id
        chunk_size = request.chunk_size
        chunk_start = chunk_id * chunk_size
        chunk_end = min(chunk_start + chunk_size, update_size)
        data_hex = version["content"]
        data_bytes = bytearray.fromhex(data_hex)
        chunk = data_bytes[chunk_start:chunk_end]
        if chunk.hex() == "":
            self.get_logger().info("End of file reached.")
            return self.__return_no_chunk(GetChunkErrorCode.END_OF_FILE)
        chunk_checksum = self.calculator.checksum(chunk)
        response.success = GetChunkErrorCode.SUCCESS.value
        response.chunk_id = chunk_id
        response.chunk_byte = chunk.hex()
        response.chunk_checksum = chunk_checksum
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FirmwareUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
