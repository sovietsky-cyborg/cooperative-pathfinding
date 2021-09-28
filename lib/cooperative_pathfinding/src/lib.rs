use std::cmp::Reverse;
use priority_queue::priority_queue::PriorityQueue;

#[derive(Clone, Copy, Default)]
pub struct Node {
    pub x: usize,
    pub y: usize,
    pub g_score: usize,
    pub f_score: usize
}

#[derive(Default)]
pub struct Agent {

    pub came_from: Vec<(Node, Node)>,
    pub g_score: Vec<(Node, usize)>,
    pub f_score: Vec<(Node, usize)>,
    pub closed_set: Vec<(Node, usize)>,

    // This priority queue will be ordered by the reverse of the highest cost
    // so, the priority of nodes exploration will depend of their accessibility cost
    pub open_set: PriorityQueue<(usize, usize), Reverse<usize>>,

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
}