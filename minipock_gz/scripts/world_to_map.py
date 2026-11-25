# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "lxml>=5",
#   "numpy>=1.26",
#   "pillow>=10",
#   "networkx>=3",
#   "pycollada>=0.8",
#   "rtree>=1",
#   "scipy>=1.11",
#   "shapely>=2",
#   "trimesh>=4",
# ]
# ///
"""Generate a Nav2 occupancy map from the collision geometry of a Gazebo SDF world.

Every collision of the world (boxes, cylinders, spheres and meshes, through `<include>` and
nested models) is placed in the world frame. Geometry entirely below `--z-min` is floor: its
footprint is free space. Geometry cut by the `[z-min, z-max]` band (the LiDAR plane) is an
obstacle: its triangles clipped to the band are rasterized, and the closed cross-sections of the
mesh inside the band are filled, so walls and furniture come out solid instead of hollow.
Everything else is unknown. The map frame is the world frame.

Example:
    uv run minipock_gz/scripts/world_to_map.py --world minipock_gz/worlds/office.sdf \
        --models minipock_gz/models --z-min 0.02 --z-max 0.1975 --resolution 0.05 \
        --out minipock_navigation/minipock_navigation2/map/office
"""

import argparse
import math
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh
from lxml import etree
from PIL import Image, ImageDraw
from shapely.geometry import Polygon

FREE = 254
OCCUPIED = 0
UNKNOWN = 205
CYLINDER_SECTIONS = 48
SPHERE_SUBDIVISIONS = 3
SECTION_LEVELS = 3
SURFACE_TOLERANCE = 0.005
POSE_VALUES = 6


@dataclass(frozen=True)
class Collision:
    """One collision geometry placed in the world frame.

    Attributes:
        name: Scoped name, for messages.
        mesh: Triangles in world coordinates.
    """

    name: str
    mesh: trimesh.Trimesh


@dataclass(frozen=True)
class Frame:
    """Placement context of the elements being read.

    Attributes:
        transform: Parent frame in the world frame (4x4).
        base: Directory against which relative URIs resolve.
        scope: Scoped name prefix, for messages.
    """

    transform: np.ndarray
    base: Path
    scope: str


def pose_matrix(element: etree._Element | None) -> np.ndarray:
    """Return the homogeneous transform of an SDF `<pose>` element.

    Args:
        element: The `<pose>` element, or None for the identity.

    Returns:
        A 4x4 transform (translation, then roll-pitch-yaw in fixed axes).

    Raises:
        ValueError: If the pose is relative to another frame than its parent.
    """
    if element is None or not (element.text or "").strip():
        return np.eye(4)
    if element.get("relative_to"):
        msg = f"pose relative_to='{element.get('relative_to')}' is not supported"
        raise ValueError(msg)
    values = [float(value) for value in element.text.split()]
    if len(values) != POSE_VALUES:
        msg = f"pose must have 6 values, got '{element.text}'"
        raise ValueError(msg)
    if element.get("degrees") == "true":
        values[3:] = [math.radians(value) for value in values[3:]]
    x, y, z, roll, pitch, yaw = values
    transform = trimesh.transformations.euler_matrix(roll, pitch, yaw, "sxyz")
    transform[:3, 3] = (x, y, z)
    return transform


class WorldReader:
    """Walk an SDF world and yield its collisions in the world frame."""

    def __init__(self, models: Path) -> None:
        """Create a reader.

        Args:
            models: Directory holding the models referenced as `model://<name>`.
        """
        self.models = models
        self._meshes: dict[Path, trimesh.Trimesh] = {}

    def collisions(self, world_file: Path) -> Iterator[Collision]:
        """Yield every collision of the world.

        Args:
            world_file: The SDF world file.

        Yields:
            The collisions, in the world frame.

        Raises:
            ValueError: If the file has no `<world>`.
        """
        world = etree.parse(world_file).getroot().find("world")
        if world is None:
            msg = f"{world_file} has no <world>"
            raise ValueError(msg)
        yield from self._children(world, Frame(np.eye(4), world_file.parent, ""))

    def _children(self, parent: etree._Element, frame: Frame) -> Iterator[Collision]:
        for model in parent.findall("model"):
            yield from self._model(model, frame, model.find("pose"), model.get("name", ""))
        for include in parent.findall("include"):
            model, model_dir = self._included(include)
            pose = include.find("pose")
            yield from self._model(
                model,
                Frame(frame.transform, model_dir, frame.scope),
                pose if pose is not None else model.find("pose"),
                include.findtext("name") or model.get("name", ""),
            )

    def _model(
        self, model: etree._Element, frame: Frame, pose: etree._Element | None, name: str
    ) -> Iterator[Collision]:
        model_frame = Frame(
            frame.transform @ pose_matrix(pose), frame.base, f"{frame.scope}{name}::"
        )
        for link in model.findall("link"):
            link_transform = model_frame.transform @ pose_matrix(link.find("pose"))
            for collision in link.findall("collision"):
                mesh = self._geometry(collision.find("geometry"), model_frame.base)
                if mesh is None:
                    continue
                mesh = mesh.copy()
                mesh.apply_transform(link_transform @ pose_matrix(collision.find("pose")))
                scoped = f"{model_frame.scope}{link.get('name')}::{collision.get('name')}"
                yield Collision(scoped, mesh)
        yield from self._children(model, model_frame)

    def _included(self, include: etree._Element) -> tuple[etree._Element, Path]:
        uri = (include.findtext("uri") or "").strip()
        model_dir = self._resolve(uri, Path())
        config = etree.parse(model_dir / "model.config").getroot()
        sdf_name = (config.findall("sdf") or [None])[-1]
        sdf_file = model_dir / (sdf_name.text.strip() if sdf_name is not None else "model.sdf")
        model = etree.parse(sdf_file).getroot().find("model")
        if model is None:
            msg = f"{sdf_file} has no <model>"
            raise ValueError(msg)
        return model, model_dir

    def _resolve(self, uri: str, base: Path) -> Path:
        if uri.startswith("model://"):
            return self.models / uri.removeprefix("model://")
        if uri.startswith("file://"):
            return Path(uri.removeprefix("file://"))
        if "://" in uri:
            msg = f"remote URI not supported: {uri}"
            raise ValueError(msg)
        return base / uri

    def _geometry(self, geometry: etree._Element | None, base: Path) -> trimesh.Trimesh | None:
        shape = None if geometry is None else next(iter(geometry), None)
        if shape is None:
            return None
        if shape.tag == "box":
            size = [float(value) for value in shape.findtext("size").split()]
            return trimesh.creation.box(extents=size)
        if shape.tag == "cylinder":
            return trimesh.creation.cylinder(
                radius=float(shape.findtext("radius")),
                height=float(shape.findtext("length")),
                sections=CYLINDER_SECTIONS,
            )
        if shape.tag == "sphere":
            return trimesh.creation.icosphere(
                subdivisions=SPHERE_SUBDIVISIONS, radius=float(shape.findtext("radius"))
            )
        if shape.tag == "mesh":
            mesh = self._mesh(self._resolve(shape.findtext("uri").strip(), base))
            scale = shape.findtext("scale")
            if scale:
                mesh = mesh.copy()
                mesh.apply_scale([float(value) for value in scale.split()])
            return mesh
        print(f"warning: <{shape.tag}> collision ignored")
        return None

    def _mesh(self, path: Path) -> trimesh.Trimesh:
        if path not in self._meshes:
            options = {"skip_materials": True} if path.suffix.lower() == ".obj" else {}
            self._meshes[path] = trimesh.load(path, force="mesh", **options)
        return self._meshes[path]


def clip_to_band(mesh: trimesh.Trimesh, z_min: float, z_max: float) -> np.ndarray:
    """Return the triangles of a mesh clipped to the horizontal band `[z_min, z_max]`.

    Args:
        mesh: The mesh, in the world frame.
        z_min: Lower bound of the band.
        z_max: Upper bound of the band.

    Returns:
        An (n, 3, 3) array of triangles, possibly empty.
    """
    vertices, faces = mesh.vertices, mesh.faces
    for normal, origin in (((0, 0, 1), (0, 0, z_min)), ((0, 0, -1), (0, 0, z_max))):
        if len(faces) == 0:
            break
        # trimesh returns (vertices, faces) or (vertices, faces, uv) depending on its version.
        vertices, faces = trimesh.intersections.slice_faces_plane(
            vertices, faces, plane_normal=normal, plane_origin=origin
        )[:2]
    return np.asarray(vertices)[np.asarray(faces, dtype=int)].reshape(-1, 3, 3)


def cross_sections(mesh: trimesh.Trimesh, z_min: float, z_max: float) -> list[Polygon]:
    """Return the closed horizontal cross-sections of a mesh inside the band.

    Args:
        mesh: The mesh, in the world frame.
        z_min: Lower bound of the band.
        z_max: Upper bound of the band.

    Returns:
        Filled polygons (with their holes) in world x, y. Open sections give nothing.
    """
    low, high = max(z_min, mesh.bounds[0][2]), min(z_max, mesh.bounds[1][2])
    polygons = []
    for z in np.linspace(low, high, SECTION_LEVELS + 2)[1:-1]:
        section = mesh.section(plane_origin=(0, 0, z), plane_normal=(0, 0, 1))
        if section is None:
            continue
        # Projecting with a pure translation keeps the world x, y in the planar path.
        to_plane = np.eye(4)
        to_plane[2, 3] = -z
        planar, _ = section.to_2D(to_2D=to_plane)
        polygons.extend(p for p in planar.polygons_full if _on_surface(mesh, p, z))
    return polygons


def _on_surface(mesh: trimesh.Trimesh, polygon: Polygon, z: float) -> bool:
    """Tell whether every edge of a section polygon lies on the mesh.

    Sections of meshes that are not watertight can close loops with edges that cross empty
    space; filling those would invent obstacles.
    """
    rings = [polygon.exterior, *polygon.interiors]
    midpoints = np.concatenate(
        [(np.asarray(ring.coords)[1:] + np.asarray(ring.coords)[:-1]) / 2 for ring in rings]
    )
    points = np.column_stack([midpoints, np.full(len(midpoints), z)])
    _, distances, _ = trimesh.proximity.closest_point(mesh, points)
    return bool(np.max(distances) < SURFACE_TOLERANCE)


class Grid:
    """Occupancy grid in the world frame, row 0 at the lowest y."""

    def __init__(self, bounds: np.ndarray, resolution: float, margin: float) -> None:
        """Create a grid covering `bounds` plus a margin.

        Args:
            bounds: [[min x, min y], [max x, max y]] in meters.
            resolution: Cell size in meters.
            margin: Extra unknown border in meters.
        """
        self.resolution = resolution
        low = np.floor((bounds[0] - margin) / resolution) * resolution
        high = np.ceil((bounds[1] + margin) / resolution) * resolution
        self.origin = low
        self.width, self.height = (int(value) for value in np.round((high - low) / resolution))
        self.floor = Image.new("1", (self.width, self.height))
        self.obstacles = Image.new("1", (self.width, self.height))

    def draw(self, image: Image.Image, triangles: np.ndarray) -> None:
        """Rasterize triangles projected on the ground plane.

        Args:
            image: `self.floor` or `self.obstacles`.
            triangles: (n, 3, 3) triangles in world coordinates.
        """
        draw = ImageDraw.Draw(image)
        cells = (triangles[:, :, :2] - self.origin) / self.resolution - 0.5
        for triangle in cells:
            points = [tuple(point) for point in triangle]
            draw.polygon(points, fill=1, outline=1)

    def fill(self, image: Image.Image, polygon: Polygon) -> None:
        """Rasterize a filled polygon, keeping its holes empty.

        Args:
            image: `self.floor` or `self.obstacles`.
            polygon: Polygon in world coordinates.
        """
        mask = Image.new("1", image.size)
        draw = ImageDraw.Draw(mask)
        draw.polygon(self._cells(polygon.exterior.coords), fill=1, outline=1)
        for interior in polygon.interiors:
            draw.polygon(self._cells(interior.coords), fill=0, outline=0)
        image.paste(1, mask=mask)

    def _cells(self, coords: Iterable[tuple[float, ...]]) -> list[tuple[float, float]]:
        points = (np.asarray(list(coords))[:, :2] - self.origin) / self.resolution - 0.5
        return [tuple(point) for point in points]

    def pixels(self) -> np.ndarray:
        """Return the trinary PGM pixels, first row at the highest y.

        Returns:
            A (height, width) uint8 array.
        """
        floor = np.asarray(self.floor, dtype=bool)
        obstacles = np.asarray(self.obstacles, dtype=bool)
        pixels = np.full((self.height, self.width), UNKNOWN, dtype=np.uint8)
        pixels[floor] = FREE
        pixels[obstacles] = OCCUPIED
        return np.flipud(pixels)


def write_map(grid: Grid, prefix: Path) -> None:
    """Write `<prefix>.pgm` and `<prefix>.yaml` in the Nav2 map_server format.

    Args:
        grid: The rasterized grid.
        prefix: Output path without extension.
    """
    pixels = grid.pixels()
    header = f"P5\n{grid.width} {grid.height}\n255\n".encode()
    prefix.with_suffix(".pgm").write_bytes(header + pixels.tobytes())
    origin_x, origin_y = (round(float(value), 6) for value in grid.origin)
    prefix.with_suffix(".yaml").write_text(
        f"image: {prefix.name}.pgm\n"
        "mode: trinary\n"
        f"resolution: {grid.resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )


def build_grid(collisions: list[Collision], args: argparse.Namespace) -> Grid:
    """Rasterize floors and obstacles of the collisions.

    Args:
        collisions: Collisions in the world frame.
        args: Parsed command line (`z_min`, `z_max`, `resolution`, `margin`).

    Returns:
        The filled grid.
    """
    floors = [c for c in collisions if c.mesh.bounds[1][2] < args.z_min]
    obstacles = []
    sections = []
    for collision in collisions:
        if collision.mesh.bounds[1][2] < args.z_min:
            continue
        triangles = clip_to_band(collision.mesh, args.z_min, args.z_max)
        if len(triangles):
            obstacles.append(triangles)
            sections.extend(cross_sections(collision.mesh, args.z_min, args.z_max))
    corners = [c.mesh.bounds[:, :2] for c in floors] + [
        np.array([t[:, :, :2].min(axis=(0, 1)), t[:, :, :2].max(axis=(0, 1))]) for t in obstacles
    ]
    stacked = np.stack(corners)
    bounds = np.array([stacked[:, 0].min(axis=0), stacked[:, 1].max(axis=0)])
    grid = Grid(bounds, args.resolution, args.margin)
    for collision in floors:
        grid.draw(grid.floor, collision.mesh.triangles)
    for triangles in obstacles:
        grid.draw(grid.obstacles, triangles)
    for polygon in sections:
        grid.fill(grid.obstacles, polygon)
    print(
        f"{len(floors)} floor and {len(obstacles)} obstacle collisions, "
        f"{len(sections)} closed cross-sections, "
        f"{grid.width}x{grid.height} cells, origin {grid.origin.round(3).tolist()}"
    )
    return grid


def main() -> None:
    """Parse the command line and write the map."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--world", type=Path, required=True, help="SDF world file")
    parser.add_argument("--models", type=Path, required=True, help="directory of model://")
    parser.add_argument("--z-min", type=float, default=0.02, help="band bottom, meters")
    parser.add_argument("--z-max", type=float, required=True, help="LiDAR height + 0.05 m")
    parser.add_argument("--resolution", type=float, default=0.05, help="meters per cell")
    parser.add_argument("--margin", type=float, default=0.5, help="unknown border, meters")
    parser.add_argument("--out", type=Path, required=True, help="output path, no extension")
    args = parser.parse_args()
    collisions = list(WorldReader(args.models).collisions(args.world))
    write_map(build_grid(collisions, args), args.out)


if __name__ == "__main__":
    main()
