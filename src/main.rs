mod util;

use std::error::Error;
use std::io;
use termion::{event::Key, input::MouseTerminal, raw::IntoRawMode, screen::AlternateScreen};
use tui::backend::TermionBackend;
use tui::Terminal;
use std::time::Duration;
use tui::layout::{Rect, Direction, Layout, Constraint};
use crate::util::event::{Config, Event, Events};
use tui::widgets::{Block, Borders};
use tui::widgets::canvas::{Canvas, Map, MapResolution, Line, Points, Rectangle};
use tui::style::Color;


static PATHFINDING_MAP_DATA: [usize; 400] = [
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, usize::MAX, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, usize::MAX, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, usize::MAX, 1, usize::MAX, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, usize::MAX, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, usize::MAX, usize::MAX, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, usize::MAX, 1, 1, 1, usize::MAX, usize::MAX,
    1, 1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1,  1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1,
    1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1, 1, usize::MAX, 1, 1, 1, 1, 1, 1, 1, 1
];

struct App {
}

impl App {
    fn new() -> App {
        App {
        }
    }
}

fn main() -> Result<(), Box<dyn Error>>{

    // Terminal initialization
    let stdout = io::stdout().into_raw_mode()?;
    let stdout = MouseTerminal::from(stdout);
    let stdout = AlternateScreen::from(stdout);
    let backend = TermionBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let config = Config {
        tick_rate: Duration::from_millis(250),
        ..Default::default()
    };
    let events = Events::new();
    let mut app = App::new();

    loop {
        terminal.draw(|f| {
            let chunks = Layout::default()
                .direction(Direction::Horizontal)
                .constraints([Constraint::Percentage(100), ].as_ref())
                .split(f.size());
            let canvas = Canvas::default()
                .block(Block::default().borders(Borders::ALL).title("Cooperatibe Pathfinding"))
                .paint(|ctx| {

                    for y in 0..21 {

                        ctx.draw(&Line{
                            x1: 0.,
                            y1: (y * 7) as f64,
                            x2: 20. * 7.,
                            y2: (y * 7) as f64,
                            color: Color::White,
                        });

                    }

                    for x in 0..21 {

                        ctx.draw(&Line{
                            x1: (x * 7) as f64,
                            y1: 0.,
                            x2: (x * 7) as f64,
                            y2: 20. * 7.,
                            color: Color::White,
                        });

                    }

                    for tile in &PATHFINDING_MAP_DATA {

                    }

                    for y in 0..20 {
                        for x in 0..20 {

                            if PATHFINDING_MAP_DATA[(y * 20) + x] == usize::MAX {

                                /*ctx.draw(&Points{
                                    coords: &[((x * 7 + 3) as f64, (y * 7 + 3) as f64)],
                                    color: Color::White
                                });*/
                                ctx.draw(&Rectangle{
                                    x: (x * 7 + 2) as f64,
                                    y: (y * 7 + 2) as f64,
                                    width: 2.,
                                    height: 2.,
                                    color: Color::White
                                });

                            }

                        }
                    }


                })
                .x_bounds([-90.0, 270.0])
                .y_bounds([0., 180.0]);
            f.render_widget(canvas, chunks[0]);
        })?;
        if let Event::Input(input) = events.next()? {
            if let Key::Char('q') = input {
                break;
            }
        }
    }

    Ok(())
}
