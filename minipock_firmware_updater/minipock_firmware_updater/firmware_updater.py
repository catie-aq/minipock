"""ROS 2 node serving firmware updates for the MiniPock micro-ROS board."""

from enum import Enum
from http import HTTPStatus

import rclpy
import requests
from jsonschema.exceptions import ValidationError
from minipock_msgs.msg import Version
from minipock_msgs.srv import (
    GetChunk,
    GetChunk_Response,
    TrigUpdate,
    TrigUpdate_Response,
)
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node

from minipock_firmware_updater import firmware

RELEASES_URL = "https://api.github.com/repos/catie-aq/minipock_zephyr-demo/releases"


class TrigUpdateErrorCode(Enum):
    """Error codes returned by the firmware update service."""

    SUCCESS = 0
    PARAMETER_NOT_DECLARED = 1
    INVALID_METHOD = 2
    GITHUB_FETCH_ERROR = 3
    NO_BIN_FILE = 4
    BIN_DOWNLOAD_ERROR = 5
    SCHEMA_VALIDATION_FAILED = 6


class GetChunkErrorCode(Enum):
    """Error codes returned by the firmware chunk service."""

    SUCCESS = 0
    VERSION_NOT_FOUND = 1
    END_OF_FILE = 2


class FirmwareUpdater(Node):
    """A ROS2 node that handles firmware updates for the minipock device."""

    def __init__(self) -> None:
        """Declare the parameters, fetch the release tags and create the services."""
        super().__init__("firmware_updater")
        self.declare_parameter(
            "namespace",
            "minipock_0",
            ParameterDescriptor(
                description="Namespace of the robot; the services are created under it at start.",
                type=ParameterType.PARAMETER_STRING,
                read_only=True,
            ),
        )
        self.declare_parameter(
            "method",
            "latest",
            ParameterDescriptor(
                description="How the firmware version is chosen, read at each update request.",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.stored_version = {}

        self.github_tags: list = self.__get_github_tags()
        self.allowed_methods: list = ["latest", *self.github_tags]
        self.get_logger().info(f"Allowed methods: {self.allowed_methods}")

        self.firmware_update_service = self.create_service(
            TrigUpdate, f"{namespace}/firmware_update", self.firmware_update_callback
        )

        self.firmware_chunk_service = self.create_service(
            GetChunk, f"{namespace}/firmware_update/chunk", self.firmware_chunk_callback
        )

    def __get_github_tags(self) -> list:
        response = requests.get(RELEASES_URL, timeout=10)
        if response.status_code == HTTPStatus.OK:
            releases = response.json()
            return [release["tag_name"] for release in releases]
        self.get_logger().error(f"Failed to fetch tags from GitHub: {response.status_code}")
        return []

    def __return_no_update(self, error_code: TrigUpdateErrorCode) -> TrigUpdate_Response:
        version = Version()
        version.major = 0
        version.minor = 0
        version.patch = 0
        response = TrigUpdate_Response()
        response.success = error_code.value
        response.new_version_available = False
        response.new_version = version
        response.size = 0
        return response

    def __get_method(self) -> str | TrigUpdate_Response:
        if self.has_parameter("method"):
            method = self.get_parameter("method").get_parameter_value().string_value
        else:
            self.get_logger().error("Parameter 'method' is not declared.")
            return self.__return_no_update(TrigUpdateErrorCode.PARAMETER_NOT_DECLARED)
        if method not in self.allowed_methods:
            return self.__return_no_update(TrigUpdateErrorCode.INVALID_METHOD)
        return method

    def __download_and_store_firmware(self, version: str) -> TrigUpdateErrorCode:
        url = f"{RELEASES_URL}/tags/{version}"
        download_response = requests.get(url, timeout=10)
        if download_response.status_code != HTTPStatus.OK:
            self.get_logger().error(
                f"Failed to fetch release {version} details from GitHub: "
                f"{download_response.status_code}"
            )
            return TrigUpdateErrorCode.GITHUB_FETCH_ERROR
        bin_url = firmware.find_bin_url(download_response.json())
        if bin_url is None:
            self.get_logger().error("No .bin file found in the release assets.")
            return TrigUpdateErrorCode.NO_BIN_FILE
        bin_response = requests.get(bin_url, timeout=10, stream=True)
        if bin_response.status_code != HTTPStatus.OK:
            self.get_logger().error(f"Failed to download .bin file: {bin_response.status_code}")
            return TrigUpdateErrorCode.BIN_DOWNLOAD_ERROR
        content = bytearray()
        for chunk in bin_response.iter_content(chunk_size=8192):
            if chunk:
                content.extend(chunk)
        self.stored_version[version] = firmware.stored_entry(bytes(content))
        try:
            firmware.validate_stored(self.stored_version)
        except ValidationError as e:
            self.get_logger().error(f"Stored version schema validation failed: {e.message}")
            return TrigUpdateErrorCode.SCHEMA_VALIDATION_FAILED
        return TrigUpdateErrorCode.SUCCESS

    def firmware_update_callback(
        self, request: TrigUpdate.Request, response: TrigUpdate.Response
    ) -> TrigUpdate_Response:
        """Handle firmware update requests.

        The function performs the following steps:
        1. Retrieves the update method.
        2. Determines the target firmware version based on the update method.
        3. Validates the actual and target firmware versions.
        4. Compares the actual firmware version with the target version.
        5. Downloads and stores the new firmware if an update is available.
        6. Populates the response object with the update information.

        Args:
            request: The request object containing the actual firmware version.
            response: The response object to be populated with the update information.

        Returns:
            The response object indicating whether a new firmware version is available,
            along with the new version details if applicable.
        """
        self.get_logger().info(f"{request}")

        method = self.__get_method()
        if isinstance(method, TrigUpdate_Response):
            return method

        version = firmware.select_version(method, self.github_tags)
        if not version:
            return self.__return_no_update(TrigUpdateErrorCode.SUCCESS)
        try:
            target = firmware.parse_version(version)
        except ValueError as e:
            self.get_logger().error(str(e))
            return self.__return_no_update(TrigUpdateErrorCode.INVALID_METHOD)

        actual = request.actual_version
        if not firmware.needs_update((actual.major, actual.minor, actual.patch), target):
            return self.__return_no_update(TrigUpdateErrorCode.SUCCESS)

        version = firmware.format_version(target)
        error_code: TrigUpdateErrorCode = self.__download_and_store_firmware(version)
        if error_code != TrigUpdateErrorCode.SUCCESS:
            return self.__return_no_update(error_code)
        response.new_version_available = True
        response.new_version.major, response.new_version.minor, response.new_version.patch = target
        response.size = self.stored_version[version]["size"]
        self.get_logger().info(f"New version available: {version}")
        return response

    def __return_no_chunk(self, error_code: GetChunkErrorCode) -> GetChunk_Response:
        response = GetChunk_Response()
        response.success = error_code.value
        response.chunk_id = 0
        response.chunk_byte = ""
        response.chunk_checksum = 0
        return response

    def firmware_chunk_callback(
        self, request: GetChunk.Request, response: GetChunk.Response
    ) -> GetChunk_Response:
        """Handle firmware chunk requests.

        Args:
            request (GetChunk.Request): The request object containing the chunk details.
            response (GetChunk.Response): The response object to be populated with the chunk data.

        Returns:
            GetChunk.Response: The response object containing the chunk data and status.
        """
        self.get_logger().info(f"{request}")
        requested = request.version
        formatted_version = firmware.format_version(
            (requested.major, requested.minor, requested.patch)
        )
        if formatted_version not in self.stored_version:
            self.get_logger().error(f"Version {request.version} not found.")
            return self.__return_no_chunk(GetChunkErrorCode.VERSION_NOT_FOUND)

        target_version = self.stored_version[formatted_version]
        firmware_chunk_index = request.chunk_id
        data = firmware.chunk(target_version, firmware_chunk_index, request.chunk_size)
        if not data:
            self.get_logger().info("End of file reached.")
            return self.__return_no_chunk(GetChunkErrorCode.END_OF_FILE)

        checksum = firmware.crc8(data)

        response.success = GetChunkErrorCode.SUCCESS.value
        response.chunk_id = firmware_chunk_index
        response.chunk_byte = data
        response.chunk_checksum = checksum
        self.get_logger().info(
            f"{response.chunk_id} / {target_version['size'] / request.chunk_size} - {checksum}"
        )
        return response


def main(args: list[str] | None = None) -> None:
    """Run the firmware updater node."""
    rclpy.init(args=args)
    node = FirmwareUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
