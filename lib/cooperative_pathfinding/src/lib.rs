use std::borrow::Borrow;
use std::cell::RefCell;
use std::cmp::Reverse;
use std::collections::HashMap;
use priority_queue::priority_queue::PriorityQueue;
use std::f32::consts::SQRT_2;
use std::hash::{Hash, Hasher};
use std::iter::FromIterator;
use std::rc::Rc;

pub const WINDOW_SIZE: u32 = 16;

type SpaceTimeMap = Vec<HashMap<(u32, u32), u32>>;

pub type Agents = HashMap::<u32, Rc<RefCell<Agent>>>;


#[derive(Default)]
pub struct WorldMap {

    pub data: Vec<u32>,
    pub width: u32,
    pub height: u32,

    pub space_time_map: SpaceTimeMap,

    pub agents: HashMap::<u32, Rc<RefCell<Agent>>>,

    pub log_file: HashMap<u32, Vec<String>>
}

impl WorldMap {

    pub fn new(data: Vec<u32>, width: u32, height: u32) -> WorldMap {

        WorldMap {
            data,
            width,
            height,
            space_time_map: std::iter::repeat(HashMap::<(u32, u32), u32>::new())
                .take(WINDOW_SIZE as usize)
                .collect::<SpaceTimeMap>(),

            ..Default::default()
        }
    }

    pub fn get_cost(&self, pos: (u32, u32)) -> u32 {
        self.data[(pos.1 * self.width + pos.0) as usize]
    }

    pub fn set_agent(&mut self, agent: Agent) {
        self.agents.insert(agent.id, Rc::new(RefCell::new(agent)));
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

        return if self.get_cost(node.pos) == u32::MAX {
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

#[derive(Default, Debug)]
pub struct Agent {

    pub came_from: HashMap<Node, Node>,
    pub cost_so_far: HashMap<(u32, u32), Node>,
    pub closed_set: HashMap<Node, u32>,

    // This priority queue will be ordered by the reverse of the highest cost
    // so, the priority of nodes exploration will depend of their accessibility cost
    pub open_set: PriorityQueue<Node, Reverse<u32>>,

    id: u32,
    pub name: String,
    start: Node,
    pub goal: Node,
    pub current_node: Node,
    previous_node: Node,
    next_node: Node,

    path: Vec<Node>,
    pub portion_path: Vec<Node>,

    is_walking: bool
}

impl Agent {

    pub fn new(id: u32, name: &str) -> Agent{
        Agent {
            id,
            name: name.into(),
            portion_path : Vec::with_capacity(WINDOW_SIZE as usize),
            // is_walking: false,
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

                        let score = node.f_score;

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

                        let el = map.get_cost((x, y));
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

    fn process_neighbors(&mut self, current_pos: Node, next_best: Node, map: &WorldMap, time: u32) -> Node {

        let mut best_neighbor = current_pos;

        /* If this is the case, get availables neighbors,
        if there is no match in the cost hashmap,
        we reload the partial search to expand new nodes
        */
        for neighbor_pos in map.get_neighbors(current_pos) {
            if neighbor_pos != next_best.pos {
                if map.space_time_map[time as usize].get(&neighbor_pos).is_none() {
                    let neighbor = match self.cost_so_far.get(&neighbor_pos) {
                        None => {
                            self.get_true_distance_heuristic(map,
                                                             Node::from((neighbor_pos.0, neighbor_pos.1, map.get_cost(neighbor_pos))),
                                                             self.goal
                            );
                            self.cost_so_far.get(&neighbor_pos).unwrap()
                        }
                        Some(node) => {
                            node
                        }
                    };

                    if neighbor.f_score <= current_pos.f_score && map.space_time_map[time as usize].get(&neighbor.pos).is_none() {
                        best_neighbor = *neighbor;
                    }
                }
            }
        }
        best_neighbor

    }

    /* Will calculate the path depending of agents position in the space-time map */
    pub fn set_portion_path(&mut self, map: &mut WorldMap, agents: &Agents) {

        self.is_walking = true;

        let mut next_best;

        for i in 0..WINDOW_SIZE {

            /* If goal already found, agent stay in place and block the tile*/
            if self.current_node == self.goal {

                self.portion_path.push(self.current_node);
                map.space_time_map[i as usize].insert(self.current_node.pos, self.id);
                self.is_walking = false;
                map.data[(self.current_node.pos.1 * map.width + self.current_node.pos.0) as usize] = 100;

            } else {
                next_best = self.came_from[&self.current_node];
                // println!("if agent {:?} at next pos {:?}, at time {:?}", self.name, next_best.pos, i);

                /* This Node is already occupied by another agent ? (excepted current) */
                if !map.space_time_map[i as usize].get(&next_best.pos).is_none() && map.space_time_map[i as usize].get(&next_best.pos).unwrap() != &self.id {

                    let best_neighbor = self.process_neighbors(self.current_node, next_best, map, i);

                    map.space_time_map[i as usize].insert(best_neighbor.pos, self.id);
                    next_best = best_neighbor;

                    // println!("agent {:?} had chosen {:?} neighbor as next node with cost {:?}", self.name, best_neighbor.pos, best_neighbor.f_score);

                /* Otherwise, we test if another agent get the risk to overlap current */
                } else if i > 0 && !map.space_time_map[(i - 1) as usize].get(&next_best.pos).is_none()  {

                    let other_agent = map.space_time_map[(i - 1) as usize].get(&next_best.pos).unwrap();
                    if !map.space_time_map[i as usize].get(&self.current_node.pos).is_none()
                        && other_agent == map.space_time_map[i as usize].get(&self.current_node.pos).unwrap() {

                        let best_neighbor = self.process_neighbors(self.current_node, next_best, map, i);

                        map.space_time_map[i as usize].insert(best_neighbor.pos, self.id);
                        next_best = best_neighbor;
                    }

                } else {
                    map.space_time_map[i as usize].insert(next_best.pos, self.id);
                }

                self.portion_path.push(next_best);
                self.current_node = next_best;

                // println!("agent {:?} choice at time {:?} is {:?}", self.name, i, self.current_node.pos);
            }

        }
        self.portion_path.reverse();
    }

    /* Will calculate the g_score by running a Reverse Resumable A* */
    pub fn get_true_distance_heuristic(&mut self, map: &WorldMap, agent_start: Node, agent_goal: Node) -> bool {

        let mut start = agent_goal;
        let goal =  agent_start;

        if map.is_obstacle(goal) {
            return false;
        }

        start.g_score = 0;
        start.f_score = WorldMap::manhattan_distance(start, goal);

        self.cost_so_far.insert(start.pos, start);
        self.open_set.push(start, Reverse(WorldMap::manhattan_distance(start, goal)));

        while let Some((current, Reverse(_current_cost))) = self.open_set.pop() {

            self.closed_set.insert(current, current.f_score);

            for mut next_pos in map.get_neighbors(current) {

                let mut next: Node = Default::default();

                match self.cost_so_far.get(&next_pos) {
                    None => {
                        next = Node {
                            pos: next_pos,
                            g_score: u32::MAX,
                            f_score: u32::MAX
                        };
                        self.cost_so_far.insert(next.pos, next);

                    }
                    Some(_) => {}
                }
                if !map.is_obstacle(next) {

                    let new_cost = {

                      /*  We take the current node cost incremented
                        from the cost of next node
                        If diagonal, add an extra cost for traversing */
                        if current.pos.0 != next.pos.0 && current.pos.1 != next.pos.1 {
                            current.g_score + f32::floor(map.get_cost(next.pos) as f32 * SQRT_2) as u32
                        } else {
                            current.g_score + map.get_cost(next.pos)
                        }
                    };

                    if self.closed_set.get(&next).is_none() || new_cost <= self.cost_so_far[&next.pos].f_score {

                        self.closed_set.insert(next, next.g_score);

                        next.g_score = new_cost;
                        next.f_score = new_cost + WorldMap::manhattan_distance(current, goal);

                        *self.cost_so_far.get_mut(&next.pos).unwrap() = next;
                        self.came_from.insert(next, current);

                        //Update priority queue with this new cost
                        self.open_set.push_decrease(next, Reverse(next.f_score));

                    }

                }else{
                    self.closed_set.insert(next, u32::MAX);
                }
            }

            if goal.pos == current.pos {
                break;
            }
        }
        true
    }
}


impl PartialEq for Agent {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
    }
}


