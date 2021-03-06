use std::f32::consts::PI;

use glium::{Display, Frame, Program, Surface, VertexBuffer};
use glium::glutin::dpi::PhysicalSize;
use rand::Rng;
use rand::prelude::ThreadRng;
use vecmath::Matrix4;

use crate::graphics::*;
use crate::data::*;
use crate::systems::*;
use crate::{AGENT_COUNT, AGENT_SIZE, INITIAL_DISPLAY_SIZE};

pub struct App {
    pub display: Display,
    pub display_size: PhysicalSize<u32>,
    
    pub perspective: Matrix4<f32>,
    pub shader: Program,
    pub agent_mesh: Mesh,
    pub instance_buffer: VertexBuffer<Transform>,

    pub components: Components,
}

pub struct Components {
    pub directions: Vec<Forward>,
    pub positions: Vec<Position>,
    pub transforms: Vec<Transform>,
}

fn get_random_positions(count: usize, rng: &mut ThreadRng) -> Vec<Position> {
    let mut positions = Vec::with_capacity(count);

    for _ in 0..count {
        positions.push(Position {
            value: [
                rng.gen_range(0.0..INITIAL_DISPLAY_SIZE[0] as f32),
                rng.gen_range(0.0..INITIAL_DISPLAY_SIZE[1] as f32),
            ]
        });
    }

    positions
}

fn get_random_directions(count: usize, rng: &mut ThreadRng) -> Vec<Forward> {
    let mut forwards = Vec::with_capacity(count);

    const TWO_PI: f32 = PI * 2.0;
    let mut angle: f32;

    for _ in 0..count {
        angle = rng.gen_range(0.0..TWO_PI);

        forwards.push(Forward {
            direction: [angle.cos(), angle.sin()]
        });
    }

    forwards
}

impl App {
    pub fn new(display: Display) -> App {
        let shader = load_program(
            &display,
            "shaders/vertex.glsl",
            "shaders/fragment.glsl"
        );

        let (vertices, indices) = create_agent_shape(
            AGENT_SIZE, 
            [1.0, 1.0, 1.0]
        );
        let agent_mesh = create_mesh(&display, &vertices, &indices);

        let mut rng = rand::thread_rng();

        let components = Components {
            directions: get_random_directions(AGENT_COUNT, &mut rng),
            positions: get_random_positions(AGENT_COUNT, &mut rng),
            transforms: vec![default_transform(); AGENT_COUNT]
        };

        let instance_buffer = VertexBuffer::dynamic(
            &display, 
            &components.transforms
        ).unwrap();

        App {
            display,
            display_size: PhysicalSize {
                width: INITIAL_DISPLAY_SIZE[0],
                height: INITIAL_DISPLAY_SIZE[1]
            },

            perspective: perspective(
                INITIAL_DISPLAY_SIZE[0], 
                INITIAL_DISPLAY_SIZE[1]
            ),
            shader,
            agent_mesh,
            instance_buffer,

            components
        }
    }


    pub fn render(&self, target: &mut Frame) {
        self.instance_buffer.write(&self.components.transforms);

        target.draw(
            (&self.agent_mesh.v_buffer, self.instance_buffer.per_instance().unwrap()),
            &self.agent_mesh.i_buffer,
            &self.shader,
            &uniform! {
                perspective: self.perspective,
            },
            &Default::default()
        ).unwrap();
    }

    pub fn update(&mut self, dt: f32) {
        boid_system(&self.components.positions, &mut self.components.directions);

        forward_system(dt, 50.0, &mut self.components.positions, &self.components.directions);

        wrap_screen_system(&mut self.components.positions, &self.display_size);

        caluclate_transform_system(
            &mut self.components.transforms, 
            &self.components.positions, 
            &self.components.directions
        );
    }

    pub fn on_window_resize(&mut self, size: &PhysicalSize<u32>) {
        self.display_size = *size;

        self.perspective = perspective(
            self.display_size.width, 
            self.display_size.height
        );
    }
}