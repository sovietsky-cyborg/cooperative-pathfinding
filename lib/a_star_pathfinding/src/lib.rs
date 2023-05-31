use std::cmp::Reverse;
use priority_queue::PriorityQueue;
use std::f32::consts::SQRT_2;
use std::collections::HashMap;

pub type Agents = HashMap::<u32, Agent>;

#[derive(Default)]
pub struct Agent {
    pub id: u32,
    pub name: String,
    pub path: Vec<(u32, u32)>,
    pub start_node: (u32, u32),
    pub end_node: (u32, u32),
    pub current_node: (u32, u32)
}

impl Agent{
    pub fn new(id: u32, name: &str, start_node:(u32, u32), end_node: (u32, u32)) -> Agent {
        Agent{
            id,
            name: name.into(),
            start_node,
            end_node,
            current_node: start_node,
            ..Default::default()
        }
    }

    pub fn astar_search(&mut self, map: &AStarMap) -> Vec<(u32, u32)> {
        AStarPathfinder::new().reconstruct_path(self.start_node, self.end_node, &map)
    }
}

#[derive(Default)]
pub struct AStarMap {
    pub data: Vec<u32>,
    width: u32,
    height: u32
}

impl AStarMap {
    pub fn new(
        data: Vec<u32>,
        width: u32,
        height: u32
    ) -> AStarMap {
       AStarMap{
            data,
            width,
            height,
       }
    }

    //Get nodes neighbors (N,W,E,S)
    fn get_neighbors(&self, position: (u32, u32)) -> Vec<(u32, u32)> {

        let mut neighbors =  Vec::<(u32, u32)>::new();

        // NE
        if position.0 < self.width - 1 && position.1 < self.height - 1{
            neighbors.push((position.0 + 1, position.1 + 1));
        }

        // SW
        if position.0 > 0 && position.1 > 0 {
            neighbors.push((position.0 - 1, position.1 - 1));
        }

        // NW
        if position.0 > 0 && position.1 < self.height - 1{
            neighbors.push((position.0 - 1, position.1 + 1));
        }

        // SE
        if position.0 < self.width - 1 && position.1 > 0{
            neighbors.push((position.0 + 1, position.1 - 1));
        }
        // E
        if position.0 < self.width - 1 {
            neighbors.push((position.0 + 1, position.1));
        }
        // W
        if position.0 > 0 {
            neighbors.push((position.0 - 1, position.1));
        }
        // N
        if position.1 < self.height - 1 {
            neighbors.push((position.0, position.1 + 1));
        }
        // W
        if position.1 > 0 {
            neighbors.push((position.0, position.1 - 1));
        }

        neighbors
    }

    // Get tuple position as an index for map lookup
    fn get_index_position(&self, x: u32, y: u32) -> usize {
        if y == 0 {
            x as usize
        } else if x == 0 {
            (y * self.width as u32) as usize
        } else {
            ((y * self.width as u32) + x) as usize
        }
    }
}

#[derive(Default)]
pub struct AStarPathfinder {
    pub path: Vec<(u32, u32)>,
    prev: Vec<(u32, u32)>,
    visited: Vec<bool>,
}

impl AStarPathfinder {

    pub fn new(
    ) -> AStarPathfinder {

        AStarPathfinder{
            ..Default::default()
        }

    }

    pub fn tuple_u32_as_i32 (tuple: (u32, u32)) -> (i32, i32) {
        (tuple.0 as i32, tuple.1 as i32)
    }


    // We use Manhattan distance to calculate
    // right angle distance between start and goal
    fn heuristic(a: (i32, i32), b: (i32, i32)) -> usize {
        (i32::abs(a.0 - b.0) + i32::abs(a.1 - b.1)) as usize
    }

    pub fn reconstruct_path(&mut self, start: (u32, u32), goal: (u32, u32), map: &AStarMap) -> Vec<(u32, u32)> {

        if map.data[map.get_index_position(goal.0, goal.1) as usize] < u32::MAX {
            let mut path = Vec::<(u32, u32)>::new();

            let _ = self.search(start, goal, map);
            let mut i = goal;

            path.push(i);

            while i != start {
                let i_index = map.get_index_position(i.0, i.1) as usize;
                path.push(self.prev[i_index]);
                i = self.prev[i_index];
            }

            path
        }else{
            Vec::<(u32, u32)>::new()
        }
    }

    pub fn search(&mut self, start: (u32, u32), goal: (u32, u32), map: &AStarMap) -> u32 {

        let start_index = map.get_index_position(start.0 , start.1);

        // if goal position is equal to u32::MAX, the goal is unreachable
        if map.data[start_index] == u32::MAX {
            return u32::MAX;
        }

        // This priority queue will be ordered by the reverse of the highest cost
        // so, the priority of nodes exploration will depend of their accessibility cost
        let mut open_list: PriorityQueue<(u32, u32), Reverse<usize>> = PriorityQueue::new();

        //Represent the nodes which have already been visited
        self.visited = std::iter::repeat(false)
            .take(map.data.len())
            .collect::<Vec<bool>>();

        //This Array contains the costs to visiting each tile,
        //they are initialy set to 'infinity'
        let mut cost_so_far = std::iter::repeat(usize::MAX)
            .take(map.data.len())
            .collect::<Vec<usize>>();

        self.prev = std::iter::repeat((0, 0))
            .take(map.data.len())
            .collect::<Vec<(u32, u32)>>();

        self.visited[start_index] = true;
        cost_so_far[start_index] = 0;
        open_list.push(start, Reverse(0));

        while let Some((current, Reverse(_current_cost))) = open_list.pop() {

            let current_pos = map.get_index_position(current.0, current.1);

            // Once we find the correct node,
            // return current pos
            if current == goal {
                return current_pos as u32;
            }

            for next in map.get_neighbors(current) {

                let next_pos = map.get_index_position(next.0, next.1);

                let mut new_cost = 0;

                // We take the current node cost incremented
                // from the cost of next node
                // If diagonal, add an extra cost for traversing
                if current.0 != next.0 && current.1 != next.1 {
                    new_cost = cost_so_far[current_pos] + (map.data[next_pos] as f32 * SQRT_2) as usize;
                }else{
                    new_cost = cost_so_far[current_pos] + map.data[next_pos] as usize;
                }

                // Node duplication detection
                if !self.visited[next_pos] || new_cost < cost_so_far[next_pos] {

                    self.visited[next_pos] = true;

                    // If this new cost is lesser than actual one,
                    // update cost map with this cost + heuristic cost (Manhattan distance)
                    let heuristic = AStarPathfinder::heuristic(
                        AStarPathfinder::tuple_u32_as_i32(goal),
                        AStarPathfinder::tuple_u32_as_i32(next));
                    cost_so_far[next_pos] = new_cost + heuristic;
                    self.prev[next_pos] = current;

                    //Update priority queue with this new cost
                    open_list.push_decrease(next, Reverse(new_cost));
                    /* match open_list.get(&next) {
                        Some((item, priority)) => {
                            open_list.push_decrease(*item, Reverse(new_cost));
                        },
                        None => {
                            open_list.push(next, Reverse(new_cost));
                        }
                    }*/
                }
            }
        }
        u32::MAX
    }
}