
bracket_terminal::add_wasm_support!();
use std::collections::HashMap;
use bracket_pathfinding::prelude::*;
use bracket_random::prelude::*;
use bracket_terminal::prelude::*;
use cooperative_pathfinding::{Agent, get_true_distance_heuristic, Node, WINDOW_SIZE, WorldMap};


static PATHFINDING_MAP_DATA: [u32; 1600] = [
    u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1,  1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1,  1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, u32::MAX, u32::MAX, u32::MAX, 1, 1, 1, 1, 1, 1, u32::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
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

struct State {
    map: Vec<TileType>,
    agents: Vec<Agent>,
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
    pub fn new(agents: Vec<Agent>, world_map: WorldMap) -> State {
        let mut state = State {
            map: vec![TileType::Floor; 40 * 40],
            agents,
            world_map,
            steps: 0
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
        match ctx.key {
            None => {}
            Some(key) => {
                if key == VirtualKeyCode::Return && self.steps < WINDOW_SIZE {

                    for i in 0..self.agents.len() {
                        let mut agent = &mut self.agents[i];
                        if self.steps % WINDOW_SIZE == 0 && self.steps < WINDOW_SIZE{
                            get_true_distance_heuristic(&mut agent, &self.world_map);
                            agent.set_portion_path(&mut self.world_map);
                        }

                        agent.current_node = agent.portion_path.pop().unwrap();
                    }

                    // Clear the screen
                    draw_batch.cls();

                    let mut block = TextBlock::new(HEIGHT, 0, 80, 25);

                    let mut buf = TextBuilder::empty();
                    buf.ln()
                        .fg(RGB::named(YELLOW))
                        .bg(RGB::named(BLUE))
                        .centered("Hello World")
                        .fg(RGB::named(WHITE))
                        .bg(RGB::named(BLACK))
                        .ln()
                        .ln()
                        .line_wrap("The quick brown fox jumped over the lazy dog, and just kept on running in an attempt to exceed the console width.")
                        .ln()
                        .ln()
                        .line_wrap("Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.")
                        .ln().ln()
                        .fg(RGB::named(CYAN))
                        .append("FPS: ")
                        .fg(RGB::named(MAGENTA))
                        .append(&format!("{}", ctx.fps))
                        .reset();

                    block.print(&buf).expect("Text was too long");

                    block.render_to_draw_batch(&mut draw_batch);

                    self.steps += 1;

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
        for i in 0..self.agents.len() {
            let agent = &self.agents[i];
            draw_batch.print_color(
                Point::new(agent.current_node.pos.0, agent.current_node.pos.1),
                &agent.name,
                ColorPair::new(RGB::from_f32(1., 0., 0.), RGB::from_f32(0., 0., 0.)),
            );
        }


        // Submit the rendering
        draw_batch.submit(0).expect("Batch error");
        render_draw_buffer(ctx).expect("Render error");
    }
}

fn main() -> BError {

    let mut world_map = WorldMap::new(Vec::from(PATHFINDING_MAP_DATA), WIDTH as u32, HEIGHT as u32);

    let mut agents = Vec::<Agent>::new();

    let mut agent_1 = Agent::new(1, "a".into());
    agent_1.set_start(Node {pos: (1, 1), g_score: 0, f_score: 0 });
    agent_1.set_goal(Node {pos: (1, 6), g_score: 0, f_score: 0 });

    let mut agent_2 = Agent::new(2, "b".into());
    agent_2.set_start(Node {pos: (1, 5), g_score: 0, f_score: 0 });
    agent_2.set_goal(Node{ pos: (1, 1), g_score: u32::MAX, f_score: 0 });


/*    get_true_distance_heuristic(&mut agent_1, &world_map);
    agent_1.print_heuristic(&world_map);
    agent_1.set_portion_path(&mut world_map);*/

    agents.push(agent_1);
    agents.push(agent_2);

    let context = BTermBuilder::simple(WIDTH + 40 , HEIGHT).unwrap()
        .with_title("Collaborative Pathfinding (WHCA*)")
        .with_dimensions(256, 192)
        .build()?;

    let gs = State::new(agents, world_map);

    main_loop(context, gs)
}