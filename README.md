# Rubicon
quadruped + applications (in landscaping and construction)
- Painting walls
- Putting plaster
- Mounting tiles
- Cutting grass(fotovolatic parks, campuses, residential)
- Collecting ciggaret buds
- Sparying plants
- Planting seeds
  
## Stack
- Language: Rust (control, drivers, messaging), TypeScript (app)
- IPC: iceoryx2 (zero-copy shared memory between processes)
- Sim: MuJoCo
- Actuators: RobStride over CAN

## Planning 

1. Control one actuator over can bus, test left righ and multiple option that you can do 
2. Link to actuators and control both of them
3. Make a gait generator for one leg to make it look like it walks 
