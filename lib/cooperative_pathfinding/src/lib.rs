use std::cmp::Reverse;
use std::collections::HashMap;
use priority_queue::priority_queue::PriorityQueue;
use std::f32::consts::SQRT_2;

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
        self.data[(node.y * self.data.width + node.x) as usize]
    }

    pub fn get_neighbors(&self, position: Node) -> Vec<Node> {

        let mut neighbors =  Vec::<Node>::new();
        let x = position.x;
        let y = position.y;

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
        if x < self.height - 1 {
            neighbors.push(Node::from((x, y + 1, u32::MAX)));
        }
        // W
        if y > 0 {
            neighbors.push(Node::from((x, y - 1, u32::MAX)));
        }

        neighbors

    }

    // We use Manhattan distance to calculate
    // right angle distance between start and goal
    pub fn manhattan_distance(a: Node, b: Node) -> u32 {
        (i32::abs(a.x - b.x) + i32::abs(a.y - b.y))
    }
}

#[derive(Clone, Copy, Default)]
pub struct Node {
    pub x: u32,
    pub y: u32,
    pub g_score: u32,
    pub f_score: u32
}

impl From<(u32, u32, u32)> for Node {
    fn from(item: (u32, u32, u32)) -> Self {
        Node{
            x: item.0,
            y: item.1,
            g_score: item.2,
            f_score: 0
        }
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> Self {
        self.x == other.x || self.y == other.y
    }
}

#[derive(Default)]
pub struct Agent {

    pub came_from: HashMap<Node, Node>,
    pub g_score: HashMap<Node, u32>,
    pub f_score: HashMap<Node, u32>,
    pub closed_set: HashMap<Node, usize>,
    // This priority queue will be ordered by the reverse of the highest cost
    // so, the priority of nodes exploration will depend of their accessibility cost
    pub open_set: PriorityQueue<Node, Reverse<usize>>,

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


    pub fn get_true_distance_heuristic(&mut self, agent : Agent, map: WorldMap, goal: Node) -> bool {

        let mut start = goal;
        let goal =  agent.start;

        let is_goal_found = false;

        if map.get_cost(goal) == u32::MAX {
            false
        }

        agent.g_score.insert(start, 0);
        agent.f_score.insert(start, 0);
        self.open_set.push(start, Reverse(0));

        while let Some((current, Reverse(_current_cost))) = self.open_set.pop() {

            if current == goal {
                true
            }

            for next in map.get_neighbors(current) {
                match self.came_from.get(&neighbor) {
                    None => {
                        self.came_from.insert(current, neighbor);
                        self.g_score.insert(neighbor, u32::MAX);
                        self.f_score.insert(current, u32::MAX);
                    }
                }

                let new_cost = {
                    // We take the current node cost incremented
                    // from the cost of next node
                    // If diagonal, add an extra cost for traversing
                    if current.x != neighbor.x && current.x != next.y {
                        current.g_score + (map.get_cost(next) as f32 * SQRT_2) as u32
                    }else{
                        current.g_score + map.get_cost(next)
                    }
                };

                if agent.closed_set.get(neighbor).is_none() && new_cost < neighbor.g_score {

                    next.g_score = new_cost;
                    next.f_score = next.g_score + WorldMap::manhattan_distance(current, next);

                    //Update priority queue with this new cost
                    self.open_set.push_decrease(next, next.f_score);

                }
            }
        }
        false
    }
}
