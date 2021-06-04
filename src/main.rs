#[macro_use]
extern crate glium;

mod graphics;
mod app;
mod systems;
mod data;

use std::time::{Duration, Instant};

use app::App;
use glium::Surface;
use glium::glutin::event::{Event, WindowEvent};
use glium::glutin::event_loop::{ControlFlow, EventLoop};
use graphics::create_display;

const BG: [f32; 4] = [0.1, 0.1, 0.1, 1.0];

pub const INITIAL_DISPLAY_SIZE: [u32; 2] = [1280, 720];

pub const AGENT_COUNT: usize = 1000;
pub const AGENT_SIZE: f32 = 10.0;

pub const CELL_SIZE: f32 = 100.0;

fn main() {
    let event_loop = EventLoop::new();
    let display = create_display(
        &event_loop,
        INITIAL_DISPLAY_SIZE[0],
        INITIAL_DISPLAY_SIZE[1]
    );

    let mut app = App::new(display);

    // Approx 60 FPS
    let frame_time = Duration::from_nanos(16_666_667);

    let mut time = Instant::now();

    event_loop.run(move |event, _, control_flow| {
        let next_frame_time = Instant::now() + frame_time;

        *control_flow = ControlFlow::WaitUntil(next_frame_time);

        match event {
            Event::WindowEvent { event, .. } => match event {
                WindowEvent::CloseRequested => {
                    *control_flow = ControlFlow::Exit;
                }
                WindowEvent::KeyboardInput { input, .. } => {
                    app.on_keyboard(input);
                }
                WindowEvent::CursorMoved { position, .. } => {
                    app.on_mouse_move(&position);
                }
                WindowEvent::Resized(size) => {
                    app.on_window_resize(&size);
                }
                _ => {},
            }
            Event::MainEventsCleared => {
                let new_time = Instant::now();
                let delta = new_time.duration_since(time).as_secs_f32();

                time = new_time;

                // Logic
                app.update(delta);
    
                // Graphics
                let mut target = app.display.draw();
                target.clear_color(BG[0], BG[1], BG[2], BG[3]);
                app.render(&mut target);
                target.finish().unwrap();
            },
            _ => (),
        }
    });
}
