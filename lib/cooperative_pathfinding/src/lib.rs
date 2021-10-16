use std::cmp::Reverse;
use std::collections::HashMap;
use priority_queue::priority_queue::PriorityQueue;
use std::f32::consts::SQRT_2;
use std::hash::{Hash, Hasher};
use std::iter::FromIterator;

const WINDOW_SIZE: u32 = 8;

type SpaceTimeMap<'a> = Vec<HashMap<(u32, u32), &'a str>>;

#[derive(Default)]
pub struct WorldMap<'a> {
    pub data: Vec<u32>,
    pub width: u32,
    pub height: u32,
    space_time_map: SpaceTimeMap<'a>
}

impl<'a> WorldMap<'a> {

    pub fn new(data: Vec<u32>, width: u32, height: u32) -> WorldMap<'a> {
        WorldMap {
            data,
            width,
            height,
            space_time_map: std::iter::repeat(HashMap::new())
                .take(WINDOW_SIZE as usize)
                .collect::<SpaceTimeMap<'a>>(),

            ..Default::default()
        }
    }

    pub fn get_cost(&self, node: Node) -> u32 {
        self.data[(node.pos.1 * self.width + node.pos.0) as usize]
    }

    pub fn get_neighbors(&self, position: Node) -> Vec<(u32, u32)> {

        let mut neighbors =  Vec::<(u32, u32)>::new();
        let x = position.pos.0;
        let y = position.pos.1;

        // NE
        if x < self.width - 1 && y < self.height - 1{
            neighbors.push((x + 1, y + 1));
        }
        // SW
        if x > 0 && y > 0 {
            neighbors.push((x - 1, y - 1));
        }
        // NW
        if x > 0 && y < self.height - 1{
            neighbors.push((x - 1, y + 1));
        }
        // SE
        if x < self.width - 1 && y > 0{
            neighbors.push((x + 1, y - 1));
        }
        // E
        if x < self.width - 1 {
            neighbors.push((x + 1, y));
        }
        // W
        if x > 0 {
            neighbors.push((x - 1, y));
        }
        // N
        if y < self.height - 1 {
            neighbors.push((x, y + 1));
        }
        // S
        if y > 0 {
            neighbors.push((x, y - 1));
        }

        neighbors

    }

    pub fn is_obstacle(&self, node: Node) -> bool {

        return if self.get_cost(node) == u32::MAX {
            true
        } else {
            false
        }

    }

    pub fn get(&self, x: u32, y: u32) -> u32 {
        self.data[(y * self.width + x) as usize]
    }

    // We use Manhattan distance to calculate
    // right angle distance between start and goal
    pub fn manhattan_distance(a: Node, b: Node) -> u32 {
        (i32::abs(a.pos.0 as i32 - b.pos.0 as i32) + i32::abs(a.pos.1 as i32 - b.pos.1 as i32)) as u32
    }
}

#[derive(Debug, Clone, Copy, Default, Eq)]
pub struct Node {
    pub pos :(u32, u32),
    pub g_score: u32,
    pub f_score: u32
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
         self.pos == other.pos
    }
}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.pos.hash(state);
    }
}

impl From<(u32, u32, u32)> for Node {
    fn from(item: (u32, u32, u32)) -> Self {
        Node{
            pos: (item.0, item.1),
            g_score: item.2,
            f_score: 0
        }
    }
}

#[derive(Default)]
pub struct Agent {

    pub came_from: HashMap<Node, Node>,
    pub cost_so_far: HashMap<(u32, u32), Node>,
    pub closed_set: HashMap<Node, u32>,

    // This priority queue will be ordered by the reverse of the highest cost
    // so, the priority of nodes exploration will depend of their accessibility cost
    pub open_set: PriorityQueue<Node, Reverse<u32>>,

    name: String,
    start: Node,
    goal: Node,
    current_node: Node,
    previous_node: Node,
    next_node: Node,

    path: Vec<Node>,
    pub portion_path: Vec<Node>,
}

impl Agent {

    pub fn new(name: &str) -> Agent{
        Agent {
            name: name.into(),
            portion_path : Vec::with_capacity(WINDOW_SIZE as usize),
            ..Default::default()
        }
    }

    pub fn set_start(&mut self, start: Node) {
        self.start = start;
        self.current_node = start;
        self.path.push(start);
    }

    pub fn get_start(&self) -> Node {
        self.start
    }

    pub fn set_goal(&mut self, goal: Node) {
        self.goal = goal;
    }

    pub fn get_goal(&self) -> Node {
        self.goal
    }

    pub fn print_heuristic(&self, map: &WorldMap) {

        for y in 0..map.height {
            for x in 0..map.width {
                let score = match self.cost_so_far.get(&(x, y)) {
                    Some(node) => {

                        let score = node.g_score;

                        if score == u32::MAX {
                            print!("  #");
                        }else{
                            if score < 10 {
                                print!("  {:?}", score);
                            } else {
                                print!(" {:?}", score);
                            }
                        }
                    }
                    None => {

                        let el = map.get_cost(Node::from((x, y, 0)));
                        if el == u32::MAX {
                            print!("  #",);
                        }else{
                            if el < 10 {
                                print!("  {:?}", el);
                            } else {
                                print!(" {:?}", el);
                            }
                        }
                    }
                };
            }
            print!("\n");
        }
    }

    /* Will calculate the path depending of agents position in the space-time map */
    pub fn set_portion_path<'a>(&'a mut self, map: &mut WorldMap<'a>) {

        let current = self.goal;
       /* let prev = current; */
        println!("map.space_time_map {:?}", map.space_time_map);
        let mut next_best = self.came_from[&current];

        for i in 0..WINDOW_SIZE {

            let occupied_node = map.space_time_map[i as usize].get(&next_best.pos);

            /* This Node is already occupied by another agent ? */
            if !occupied_node.is_none() && occupied_node.unwrap() != &self.name{

                /* Otherwise, Node is already occupied by another agent on T+1 ? */
                match map.space_time_map[(i + 1) as usize].get(&next_best.pos) {
                    Some(_) => {

                        /* Node is definitely unreachable, so we check each neighbor g_cost */
                        let mut best_neighbor = current;

                        for neighbor_pos in map.get_neighbors(current) {
                            if map.space_time_map[i as usize].get(&neighbor_pos).is_none() {

                                let neighbor = self.cost_so_far.get(&neighbor_pos).unwrap() ;
                                if neighbor.f_score <= current.f_score {
                                    best_neighbor = *neighbor;
                                }
                            }
                        }
                        map.space_time_map[(i + 1) as usize].insert(best_neighbor.pos, &self.name).unwrap();
                    },
                    None => {
                        /* T+1 is available, reserve for next iteration */
                        next_best = current;
                        map.space_time_map[(i + 1) as usize].insert(next_best.pos, &self.name).unwrap();
                    }
                }
            }

            next_best = self.came_from[&current];
            self.portion_path.push(next_best);
        }
    }
}


impl PartialEq for Agent {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
    }
}

/* Will calculate the g_score by running a Reverse Resumable A* */
pub fn get_true_distance_heuristic(agent: &mut Agent, map: &WorldMap) -> bool {

    let mut start = agent.goal;
    let goal =  agent.start;

    if map.is_obstacle(goal) {
        return false;
    }

    start.g_score = 0;
    start.f_score = WorldMap::manhattan_distance(start, goal);

    agent.cost_so_far.insert(start.pos, start);
    agent.open_set.push(start, Reverse(WorldMap::manhattan_distance(start, goal)));

    while let Some((current, Reverse(_current_cost))) = agent.open_set.pop() {

        agent.closed_set.insert(current, current.f_score);

        for mut next_pos in map.get_neighbors(current) {

            let mut next: Node = Default::default();

            match agent.cost_so_far.get(&next.pos) {
                None => {
                    next = Node {
                        pos: next_pos,
                        g_score: u32::MAX,
                        f_score: u32::MAX
                    };
                    agent.cost_so_far.insert(next.pos, next);

                }
                Some(_) => {}
            }

            if !map.is_obstacle(next) {

                let new_cost = {

                    // We take the current node cost incremented
                    // from the cost of next node
                    // If diagonal, add an extra cost for traversing
                    if current.pos.0 != next.pos.0 && current.pos.1 != next.pos.1 {
                        current.g_score + (map.get_cost(next) as f32 * SQRT_2) as u32
                    } else {
                        current.g_score + map.get_cost(next)
                    }
                };

                if agent.closed_set.get(&next).is_none() || new_cost < agent.cost_so_far[&next.pos].g_score {

                    next.g_score = new_cost;
                    next.f_score = new_cost + WorldMap::manhattan_distance(current, goal);

                    agent.came_from.insert(current, next);
                    agent.closed_set.insert(next, next.f_score);
                    *agent.cost_so_far.get_mut(&next.pos).unwrap() = next;

                    //Update priority queue with this new cost
                    agent.open_set.push_decrease(next, Reverse(next.f_score));

                }

            }else{
                agent.closed_set.insert(next, u32::MAX);
            }
        }
    }
    true
}

