use std::cmp::Reverse;
use std::collections::HashMap;
use priority_queue::priority_queue::PriorityQueue;
use std::f32::consts::SQRT_2;
use std::hash::{Hash, Hasher};

const WINDOW_SIZE: u32 = 8;

pub struct WorldMap {
    pub data: Vec<u32>,
    pub width: u32,
    pub height: u32
}

impl WorldMap {

    pub fn new(data: Vec<u32>, width: u32, height: u32) -> WorldMap {
        WorldMap {
            data,
            width,
            height
        }
    }

    pub fn get_cost(&self, node: Node) -> u32 {
        self.data[(node.pos.1 * self.width + node.pos.0) as usize]
    }

    pub fn get_neighbors(&self, position: Node) -> Vec<Node> {

        let mut neighbors =  Vec::<Node>::new();
        let x = position.pos.0;
        let y = position.pos.1;

        // NE
        if x < self.width - 1 && y < self.height - 1{
            neighbors.push(Node::from((x + 1, y + 1, u32::MAX)));
        }

        // SW
        if x > 0 && y > 0 {
            neighbors.push(Node::from((x - 1, y - 1, u32::MAX)));
        }

        // NW
        if x > 0 && y < self.height - 1{
            neighbors.push(Node::from((x - 1, y + 1, u32::MAX)));
        }

        // SE
        if x < self.width - 1 && y > 0{
            neighbors.push(Node::from((x + 1, y - 1, u32::MAX)));
        }
        // E
        if x < self.width - 1 {
            neighbors.push(Node::from((x + 1, y, u32::MAX)));
        }
        // W
        if x > 0 {
            neighbors.push(Node::from((x - 1, y, u32::MAX)));
        }
        // N
        if y < self.height - 1 {
            neighbors.push(Node::from((x, y + 1, u32::MAX)));
        }
        // W
        if y > 0 {
            neighbors.push(Node::from((x, y - 1, u32::MAX)));
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

// impl Node {
//     pub fn is_obstacle(&self) -> bool {
//         if self.g_score >= u32::MAX{
//             return true;
//         }else{
//             return false;
//         }
//     }
// }

#[derive(Default)]
pub struct Agent {

    pub came_from: HashMap<Node, Node>,
    pub g_score: HashMap<Node, u32>,
    pub f_score: HashMap<Node, u32>,
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
    portion_path: Vec<Node>,
}

impl Agent {

    pub fn new(name: &str) -> Agent{
        Agent {
            name: name.into(),
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

    pub fn set_portion_path(&mut self) {

    }

    pub fn print_heuristic(&self, map: &WorldMap) {

        for y in 0..map.height - 1 {
            for x in 0..map.width - 1 {
                let score = match self.f_score.get(&Node::from((x, y, 0))) {
                    Some(el) => {
                        if *el == u32::MAX {
                            print!("inf",);
                        }else{
                            if *el < 10 {
                                print!("  {:?}", el);
                            } else {
                                print!(" {:?}", el);
                            }
                        }
                    }
                    None => {

                        let el = map.get_cost(Node::from((x, y, 0)));
                        if el == u32::MAX {
                            print!("inf",);
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
}

pub fn get_true_distance_heuristic(agent: &mut Agent, map: &WorldMap) -> bool {

    let start = agent.goal;
    let goal =  agent.start;

    // let is_goal_found = false;
    if map.is_obstacle(goal) {
        return false;
    }

    agent.g_score.insert(start, 0);
    agent.f_score.insert(start, 0);
    agent.open_set.push(start, Reverse(0));

    let mut count = 0;

    while let Some((current, Reverse(_current_cost))) = agent.open_set.pop() {

        if current.pos == goal.pos {
            return true;
        }

        agent.closed_set.insert(current, current.f_score);

        for mut next in map.get_neighbors(current) {
            match agent.came_from.get(&next) {
                None => {
                    agent.came_from.insert(current, next);
                    agent.g_score.insert(next, u32::MAX);
                    agent.f_score.insert(next, u32::MAX);

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
                        agent.g_score[&current] + map.get_cost(next)
                    }
                };

                if agent.closed_set.get(&next).is_none() && new_cost < agent.g_score[&next] {

                    next.g_score = new_cost;
                    next.f_score = new_cost + WorldMap::manhattan_distance(current, next);

                    *agent.g_score.get_mut(&next).unwrap() = next.g_score;
                    *agent.f_score.get_mut(&next).unwrap() = next.f_score;

                    //Update priority queue with this new cost
                    agent.open_set.push_decrease(next, Reverse(next.f_score));

                }
            }else{
                agent.closed_set.insert(next, u32::MAX);
            }
        }
        count = count + 1;
        if count > 10{
            return true;
        }
    }
    true
}

