bracket_terminal::add_wasm_support!();

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;
use std::env;

use bracket_pathfinding::prelude::*;
use bracket_random::prelude::*;
use bracket_terminal::prelude::*;

use cooperative_pathfinding::{Agent as Coop_A_Star_Agent, Agents as Coop_A_Star_Agents, Node, WINDOW_SIZE, WorldMap};
use a_star_pathfinding::{Agent as A_Star_Agent, Agents as A_Star_Agents, AStarMap};

static PATHFINDING_MAP_DATA: [u32; 1600] = [
    u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1,  1, u32::MAX, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,u32::MAX-1, u32::MAX-1, u32::MAX-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX-1, u32::MAX-1, u32::MAX-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX-1, u32::MAX-1, u32::MAX-1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, u32::MAX, u32::MAX, 1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, u32::MAX, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1,  1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1
];

const WIDTH: i32 = 40;
const HEIGHT: i32 = 40;


#[derive(PartialEq, Copy, Clone)]
enum TileType {
    Wall,
    Floor,
    Water
}

#[derive(PartialEq, Copy, Clone)]
enum Mode {
    Waiting,
    Moving,
}
#[derive(Default)]
struct State_Coop_A_Star {
    map: Vec<TileType>,
    agents: Coop_A_Star_Agents,
    world_map: WorldMap,
    steps: u32
}

#[derive(Default)]
struct State_A_Star {
    map: Vec<TileType>,
    agents: A_Star_Agents,
    world_map: AStarMap,
}

pub fn xy_idx(x: i32, y: i32) -> usize {
    (y as usize * WIDTH as usize) + x as usize
}

pub fn idx_xy(idx: usize) -> (i32, i32) {
    (idx as i32 % WIDTH, idx as i32 / WIDTH )
}
impl State_Coop_A_Star {
    pub fn new(world_map: WorldMap, agents: Coop_A_Star_Agents) -> State_Coop_A_Star {
        let mut state = State_Coop_A_Star {
            map: vec![TileType::Floor; 40 * 40],
            world_map,
            steps: 0,
            agents,
            ..Default::default()
        };

        for i in 0..1600 {
            if PATHFINDING_MAP_DATA[i] == u32::MAX {
                state.map[i] = TileType::Wall;
            }
            if PATHFINDING_MAP_DATA[i] == u32::MAX - 1 {
                state.map[i] = TileType::Water;
            }
        }
        state
    }
}

impl State_A_Star {

    pub fn new(world_map: AStarMap, agents: A_Star_Agents) -> State_A_Star {
        let mut state = State_A_Star {
            map: vec![TileType::Floor; 40 * 40],
            world_map,
            agents,
            // ..Default::default()
        };

        for i in 0..1600 {
            if PATHFINDING_MAP_DATA[i] == u32::MAX {
                state.map[i] = TileType::Wall;
            }
            if PATHFINDING_MAP_DATA[i] == u32::MAX - 1 {
                state.map[i] = TileType::Water;
            }
        }
        state
    }
}


// Implement the game loop
impl GameState for State_Coop_A_Star {
    #[allow(non_snake_case)]
    fn tick(&mut self, ctx: &mut BTerm) {

        // We'll use batched drawing
        let mut draw_batch = DrawBatch::new();
        draw_batch.cls();

        match ctx.key {
            None => {}
            Some(key) => {
                if key == VirtualKeyCode::Return && self.steps < WINDOW_SIZE {

                    for i in 1..self.agents.len() + 1 {

                        let mut rc = &self.agents.get(&(i as u32)).unwrap();
                        let mut agent = &mut *rc.borrow_mut();
                        // println!("agent {:?}", agent.name);

                        if self.steps % WINDOW_SIZE == 0 && self.steps < WINDOW_SIZE{
                            agent.get_true_distance_heuristic(&self.world_map, agent.get_start(), agent.get_goal(), &self.agents);
                            // println!("agent {:?} heuristic",  agent.name);
                            // agent.print_heuristic(&self.world_map);
                            agent.set_portion_path(&mut self.world_map, &self.agents);
                        }

                        agent.current_node = agent.portion_path.pop().unwrap();
                    }
                    self.steps += 1;
                }
                if key == VirtualKeyCode::Q {
                    return;
                }
            }
        };

        // Iterate the map array, incrementing coordinates as we go.
        let mut y = 0;
        let mut x = 0;
        for (i, tile) in self.map.iter().enumerate() {
            // Render a tile depending upon the tile type; now we check visibility as well!
            let mut fg;
            let mut glyph = ".";

            match tile {
                TileType::Floor => {
                    fg = RGB::from_f32(0.5, 0.5, 0.0);
                }
                TileType::Wall => {
                    fg = RGB::from_f32(0.0, 1.0, 0.0);
                    glyph = "#";
                }
                TileType::Water => {
                    fg = RGB::from_f32(0.0, 0.8, 1.0);
                    glyph = "0";
                }
            }
            draw_batch.print_color(
                Point::new(x, y),
                glyph,
                ColorPair::new(fg, RGB::from_f32(0., 0.,  0.)),
            );

            // Move the coordinates
            x += 1;
            if x > WIDTH - 1 {
                x = 0;
                y += 1;
            }
        }
        for i in 1..self.agents.len() + 1 {

            let mut rc = &self.agents.get(&(i as u32)).unwrap();
            let mut agent = &mut *rc.borrow_mut();

            draw_batch.print_color(
                Point::new(agent.current_node.pos.0, agent.current_node.pos.1),
                &agent.name,
                ColorPair::new(RGB::from_f32(1., 0., 0.), RGB::from_f32(0., 0., 0.)),
            );

            draw_batch.print_color(
                Point::new(agent.goal.pos.0, agent.goal.pos.1),
                &agent.name,
                ColorPair::new(RGB::from_f32(1., 0., 0.), RGB::from_f32(0., 1., 0.)),
            );
        }

        // Submit the rendering
        draw_batch.submit(0).expect("Batch error");
        render_draw_buffer(ctx).expect("Render error");
    }
}

impl GameState for State_A_Star {

    #[allow(non_snake_case)]
    fn tick(&mut self, ctx: &mut BTerm) {

         // We'll use batched drawing
         let mut draw_batch = DrawBatch::new();
         draw_batch.cls();

         match ctx.key {
            None => {}
            Some(key) => {
                if key == VirtualKeyCode::Return {

                    for (k, agent) in self.agents.iter_mut() {

                        let mut path = agent.astar_search(&self.world_map);
                        if path.len() > 0 {                      
                            println!("agent {} path {:?}", agent.name, path);
                            agent.current_node = path.pop().unwrap();

                            println!("agent {} path {:?}", agent.name, path);

                            println!("agent {} current_node {:?}", agent.name, agent.current_node );
                            agent.start_node = match path.last() {
                                Some(node) => {
                                    *node
                                },
                                None => {
                                    agent.current_node
                                }
                            }                       
                         }
                    }
                }
                if key == VirtualKeyCode::Q {
                    return;
                }
            }
        };


        // Iterate the map array, incrementing coordinates as we go.
        let mut y = 0;
        let mut x = 0;
        for (i, tile) in self.map.iter().enumerate() {
            // Render a tile depending upon the tile type; now we check visibility as well!
            let mut fg;
            let mut glyph = ".";

            match tile {
                TileType::Floor => {
                    fg = RGB::from_f32(0.5, 0.5, 0.0);
                }
                TileType::Wall => {
                    fg = RGB::from_f32(0.0, 1.0, 0.0);
                    glyph = "#";
                }
                TileType::Water => {
                    fg = RGB::from_f32(0.0, 0.8, 1.0);
                    glyph = "0";
                }
            }
            draw_batch.print_color(
                Point::new(x, y),
                glyph,
                ColorPair::new(fg, RGB::from_f32(0., 0.,  0.)),
            );

            // Move the coordinates
            x += 1;
            if x > WIDTH - 1 {
                x = 0;
                y += 1;
            }
        }
    
        for (k, agent) in self.agents.iter() {

            draw_batch.print_color(
                Point::new(agent.current_node.0, agent.current_node.1),
                &agent.name,
                ColorPair::new(RGB::from_f32(1., 0., 0.), RGB::from_f32(0., 0., 0.)),
            );

            draw_batch.print_color(
                Point::new(agent.end_node.0, agent.end_node.1),
                &agent.name,
                ColorPair::new(RGB::from_f32(1., 0., 0.), RGB::from_f32(0., 1., 0.)),
            );
        }

        // Submit the rendering
        draw_batch.submit(0).expect("Batch error");
        render_draw_buffer(ctx).expect("Render error");

    }
}

fn main() -> BError {

    let args: Vec<String> = env::args().collect();

    match args[1].clone().as_str() {
            "astar" => {
                println!("i have choosen A*");
                let map_data = PATHFINDING_MAP_DATA.clone();
                let a_star_map = AStarMap::new(Vec::from(map_data), WIDTH as u32, HEIGHT as u32);

                let mut agents = A_Star_Agents::new();
                let agent_1 = A_Star_Agent::new(1, "a".into(), (36, 20), (38, 22));
                let agent_2 = A_Star_Agent::new(1, "b".into(), (35, 19), (38, 23));
                let agent_3 = A_Star_Agent::new(1, "c".into(), (33, 20), (38, 24));

                agents.insert(1, agent_1);
                agents.insert(2, agent_2);
                agents.insert(3, agent_3);

                let gs = State_A_Star::new(a_star_map, agents);

                let context = BTermBuilder::simple(WIDTH , HEIGHT).unwrap()
                .with_title("Pathfinding (A*)")
                .with_dimensions(256, 192)
                .build()?;
            
                main_loop(context, gs)
            },
            "hca_star" => {
                println!("i have choosen Cooperative A*");
                let map_data = PATHFINDING_MAP_DATA.clone();

                let world_map = WorldMap::new(Vec::from(map_data), WIDTH as u32, HEIGHT as u32);
        
                let mut agents = Coop_A_Star_Agents::new();
        
                let mut agent_1 = Coop_A_Star_Agent::new(1, "a".into());
                agent_1.set_start(Node {pos: (36, 20), g_score: 0, f_score: 0 });
                agent_1.set_goal(Node {pos: (38, 22), g_score: 0, f_score: 0 });
        
                let mut agent_2 = Coop_A_Star_Agent::new(2, "b".into());
                agent_2.set_start(Node {pos: (35, 19), g_score: 0, f_score: 0 });
                agent_2.set_goal(Node{ pos: (38, 23), g_score: 0, f_score: 0 });
        
                let mut agent_3 = Coop_A_Star_Agent::new(3, "c".into());
                agent_3.set_start(Node {pos: (33, 20), g_score: 0, f_score: 0 });
                agent_3.set_goal(Node{ pos: (38, 24), g_score: 0, f_score: 0 });
        
        
                // let mut agent_1 = Agent::new(1, "a".into());
                // agent_1.set_start(Node {pos: (1, 1), g_score: 0, f_score: 0 });
                // agent_1.set_goal(Node {pos: (1, 8), g_score: 0, f_score: 0 });
        
                // let mut agent_2 = Agent::new(2, "b".into());
                // agent_2.set_start(Node {pos: (1, 5), g_score: 0, f_score: 0 });
                // agent_2.set_goal(Node{ pos: (1, 1), g_score: u32::MAX, f_score: 0 });
        
                // let mut agent_3 = Agent::new(3, "c".into());
                // agent_3.set_start(Node {pos: (5, 1), g_score: 0, f_score: 0 });
                // agent_3.set_goal(Node{ pos: (0, 10), g_score: u32::MAX, f_score: 0 });
        
        
                agents.insert(1, Rc::new(RefCell::new(agent_1)));
                agents.insert(2, Rc::new(RefCell::new(agent_2)));
                agents.insert(3, Rc::new(RefCell::new(agent_3)));
        
                let gs = State_Coop_A_Star::new(world_map, agents);

                let context = BTermBuilder::simple(WIDTH , HEIGHT).unwrap()
                .with_title("Collaborative Pathfinding (WHCA*)")
                .with_dimensions(256, 192)
                .build()?;
            
                main_loop(context, gs)
            },
            _ => {panic!("bad argument");}
        }
    

}