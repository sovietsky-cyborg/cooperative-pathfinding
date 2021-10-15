extern crate cooperative_pathfinding;
use cooperative_pathfinding::{WorldMap, Agent, Node, get_true_distance_heuristic};

use std::collections::HashMap;
use bracket_terminal::prelude::*;
use bracket_lib::prelude::*;
use std::error::Error;

static PATHFINDING_MAP_DATA: [u32; 400] = [
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, u32::MAX, u32::MAX,
    1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1
];

struct State {}

impl GameState for State {
    fn tick(&mut self, ctx: &mut BTerm) {
        ctx.print(1, 1, "Hello Bracket World");

    }
}

fn main() {

/*    let world_map = WorldMap::new(Vec::from(PATHFINDING_MAP_DATA),20, 20);

    let space_time_map: Vec<Vec<HashMap<u32, u32>>> = Vec::new();

    let mut agent_1 = Agent::new("a");
    agent_1.set_start(Node {pos: (0, 0), g_score: 0, f_score: 0 });
    agent_1.set_goal(Node {pos: (5, 19), g_score: 0, f_score: 0 });

    let mut agent_2 = Agent::new("b");
    agent_2.set_start(Node {pos: (5, 8), g_score: 0, f_score: 0 });
    agent_2.set_goal(Node{ pos: (10, 11), g_score: u32::MAX, f_score: 0 });

    get_true_distance_heuristic(&mut agent_1, &world_map);
    get_true_distance_heuristic(&mut agent_2, &world_map);
    agent_1.print_heuristic(&world_map);*/

    let context = BTermBuilder::simple80x50()
        .with_title("Hello Minimal Bracket World")
        .build();

    let gs: State = State {};
    main_loop(context, gs);
}
