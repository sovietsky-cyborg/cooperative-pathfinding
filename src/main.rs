
bracket_terminal::add_wasm_support!();
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;
use bracket_pathfinding::prelude::*;
use bracket_random::prelude::*;
use bracket_terminal::prelude::*;
use cooperative_pathfinding::{Agent, Agents, Node, WINDOW_SIZE, WorldMap};


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
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1,
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
}

#[derive(PartialEq, Copy, Clone)]
enum Mode {
    Waiting,
    Moving,
}
#[derive(Default)]
struct State {
    map: Vec<TileType>,
    agents: Agents,
    world_map: WorldMap,
    steps: u32
}

pub fn xy_idx(x: i32, y: i32) -> usize {
    (y as usize * WIDTH as usize) + x as usize
}

pub fn idx_xy(idx: usize) -> (i32, i32) {
    (idx as i32 % WIDTH, idx as i32 / WIDTH )
}
impl State {
    pub fn new(world_map: WorldMap, agents: Agents) -> State {
        let mut state = State {
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
        }
        state
    }
}

// Implement the game loop
impl GameState for State {
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
                            agent.get_true_distance_heuristic(&self.world_map, agent.get_start(), agent.get_goal());
                            // println!("agent {:?} heuristic",  agent.name);
                            agent.print_heuristic(&self.world_map);
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

      /*  let mut block = TextBlock::new(HEIGHT, 0, 80, 25);
        let mut buf = TextBuilder::empty();

        for i in 0..self.steps {

            let log_step = self.world_map.log_file.get(&i).unwrap();

            for y in 0..log_step.len() {
                buf.ln().line_wrap(&*log_step[y])
                    .ln();
            }
        }
        block.print(&buf).expect("Text was too long");

        block.render_to_draw_batch(&mut draw_batch);
*/

        // Submit the rendering
        draw_batch.submit(0).expect("Batch error");
        render_draw_buffer(ctx).expect("Render error");
    }
}

fn main() -> BError {

    let map_data = PATHFINDING_MAP_DATA.clone();

    let world_map = WorldMap::new(Vec::from(map_data), WIDTH as u32, HEIGHT as u32);

    let mut agents = Agents::new();

    let mut agent_1 = Agent::new(1, "a".into());
    agent_1.set_start(Node {pos: (36, 20), g_score: 0, f_score: 0 });
    agent_1.set_goal(Node {pos: (38, 22), g_score: 0, f_score: 0 });

    let mut agent_2 = Agent::new(2, "b".into());
    agent_2.set_start(Node {pos: (35, 19), g_score: 0, f_score: 0 });
    agent_2.set_goal(Node{ pos: (38, 23), g_score: u32::MAX, f_score: 0 });

    let mut agent_3 = Agent::new(3, "c".into());
    agent_3.set_start(Node {pos: (33, 20), g_score: 0, f_score: 0 });
    agent_3.set_goal(Node{ pos: (38, 24), g_score: u32::MAX, f_score: 0 });


/*    let mut agent_1 = Agent::new(1, "a".into());
    agent_1.set_start(Node {pos: (1, 1), g_score: 0, f_score: 0 });
    agent_1.set_goal(Node {pos: (1, 8), g_score: 0, f_score: 0 });

    let mut agent_2 = Agent::new(2, "b".into());
    agent_2.set_start(Node {pos: (1, 5), g_score: 0, f_score: 0 });
    agent_2.set_goal(Node{ pos: (1, 1), g_score: u32::MAX, f_score: 0 });

    let mut agent_3 = Agent::new(3, "c".into());
    agent_3.set_start(Node {pos: (5, 1), g_score: 0, f_score: 0 });
    agent_3.set_goal(Node{ pos: (0, 10), g_score: u32::MAX, f_score: 0 });*/


    agents.insert(1, Rc::new(RefCell::new(agent_1)));
    agents.insert(2, Rc::new(RefCell::new(agent_2)));
    agents.insert(3, Rc::new(RefCell::new(agent_3)));


    let context = BTermBuilder::simple(WIDTH , HEIGHT).unwrap()
        .with_title("Collaborative Pathfinding (WHCA*)")
        .with_dimensions(256, 192)
        .build()?;

    let gs = State::new(world_map, agents);

    main_loop(context, gs)
}