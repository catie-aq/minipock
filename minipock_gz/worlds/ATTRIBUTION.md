# Attribution of the office world

`office.sdf` and the models in `minipock_gz/models/` come from third parties.
Everything is vendored so that the simulation never downloads anything.

## World

`office.sdf` is derived from `worlds/husarion_office.sdf` of
[husarion/husarion_gz_worlds](https://github.com/husarion/husarion_gz_worlds),
commit `87271ef83672d9c9c96cb014ac64d746cacfd2c4`, Apache-2.0, by Husarion.

Changes from the original:

- world renamed `office`;
- the three ROSbot models removed (the Husarion logo is not part of this world);
- Fuel mesh and texture URLs replaced by local `model://` URIs;
- model names stripped of the `OpenRobotics/` prefix, `Shelf_1` (a fridge mesh) renamed
  `Fridge_2` to keep names unique;
- classic Gazebo `<state>`, `<gui>` and `file://media/...` material scripts removed, a
  Gazebo Sim GUI configuration added (same as `minipock_world.sdf`, camera on the office);
- system plugins renamed to `gz-sim-*`, physics step set to 10 ms like the other MiniPock
  worlds.

## Models

| Model | License | Source | Changes |
|-------|---------|--------|---------|
| `ConferenceTable` | Apache-2.0 | husarion_gz_worlds, Husarion | none |
| `Door` | Apache-2.0 | husarion_gz_worlds, Husarion | `door.stl` decimated from 127k to 23k faces |
| `Kitchen` | Apache-2.0 | husarion_gz_worlds, Husarion | `kitchen.stl` decimated from 48k to 12k faces |
| `Laptop` | Apache-2.0 | husarion_gz_worlds, Husarion | none |
| `Surfaces` | Apache-2.0 | husarion_gz_worlds, Husarion | `white_carpet.png` downscaled to 1024 px, PNGs recompressed |
| `Tv` | Apache-2.0 | husarion_gz_worlds, Husarion | none |
| `WaterDispenser` | Apache-2.0 | husarion_gz_worlds, Husarion | `water_dispenser.dae` converted to `water_dispenser.obj` + `.mtl` (same colors), decimated from 57k to 11k faces |
| `AdjTable` | CC-BY-4.0 | [Fuel OpenRobotics/AdjTable](https://app.gazebosim.org/OpenRobotics/fuel/models/AdjTable) v3, Roselle | texture downscaled to 1024 px |
| `BathroomSink` | CC-BY-4.0 | [Fuel OpenRobotics/BathroomSink](https://app.gazebosim.org/OpenRobotics/fuel/models/BathroomSink) v1, Wan Yi Seow | none |
| `CoffeeTable` | CC-BY-4.0 | [Fuel OpenRobotics/CoffeeTable](https://app.gazebosim.org/OpenRobotics/fuel/models/CoffeeTable) v1 | normal map recompressed |
| `DeskChair` | CC-BY-4.0 | [Fuel OpenRobotics/DeskChair](https://app.gazebosim.org/OpenRobotics/fuel/models/DeskChair) v1, Cole Biesemeyer | texture downscaled to 1024 px |
| `Drawer` | CC-BY-4.0 | [Fuel OpenRobotics/Drawer](https://app.gazebosim.org/OpenRobotics/fuel/models/Drawer) v1, Wan Yi Seow | none |
| `Fridge` | CC-BY-4.0 | [Fuel OpenRobotics/Fridge](https://app.gazebosim.org/OpenRobotics/fuel/models/Fridge) v1, Roselle | texture recompressed |
| `MopCart2` | CC-BY-4.0 | [Fuel OpenRobotics/MopCart2](https://app.gazebosim.org/OpenRobotics/fuel/models/MopCart2) v1, Roselle Carmen | textures downscaled to 512 px |
| `MopCart3` | CC-BY-4.0 | [Fuel OpenRobotics/MopCart3](https://app.gazebosim.org/OpenRobotics/fuel/models/MopCart3) v1, Roselle Carmen | textures downscaled to 512 px |
| `OfficeChairBlack` | CC-BY-4.0 | [Fuel OpenRobotics/OfficeChairBlack](https://app.gazebosim.org/OpenRobotics/fuel/models/OfficeChairBlack) v1, Wan Yi Seow | none |
| `SquareShelf` | CC-BY-4.0 | [Fuel OpenRobotics/SquareShelf](https://app.gazebosim.org/OpenRobotics/fuel/models/SquareShelf) v2, Wan Yi Seow | none |
| `Table` | CC0-1.0 | [Fuel OpenRobotics/Table](https://app.gazebosim.org/OpenRobotics/fuel/models/Table) v3, Nate Koenig | only `model.sdf`, `model.config` and the texture kept; classic Gazebo material scripts (`gazebo.material`) removed, as in `office.sdf` |
| `Toilet` | CC-BY-4.0 | [Fuel OpenRobotics/Toilet](https://app.gazebosim.org/OpenRobotics/fuel/models/Toilet) v1, Roselle Carmen | none |
| `TrashBin` | CC-BY-4.0 | [Fuel OpenRobotics/TrashBin](https://app.gazebosim.org/OpenRobotics/fuel/models/TrashBin) v1, Roselle Carmen | texture recompressed |
| `WhiteCabinet` | CC-BY-4.0 | [Fuel OpenRobotics/WhiteCabinet](https://app.gazebosim.org/OpenRobotics/fuel/models/WhiteCabinet) v1, Wan Yi Seow | none |
| `WoodenChair` | CC0-1.0 | [Fuel OpenRobotics/WoodenChair](https://app.gazebosim.org/OpenRobotics/fuel/models/WoodenChair) v1, Wan Yi Seow | none |

Fuel models are owned by Open Robotics; the license is the one shown on each model's Fuel
page (the downloaded archives carry no license file). Thumbnails were not copied.

- Apache-2.0: <https://www.apache.org/licenses/LICENSE-2.0>
- CC-BY-4.0: <https://creativecommons.org/licenses/by/4.0/>
- CC0-1.0: <https://creativecommons.org/publicdomain/zero/1.0/>

Thanks to Husarion for the original office world.
