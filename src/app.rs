use glium::{Display, Frame, Program, Surface, VertexBuffer};
use glium::glutin::event::KeyboardInput;
use glium::glutin::dpi::{PhysicalPosition, PhysicalSize};
use rand::Rng;
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

        let mut components = Components {
            directions: vec![Forward { direction: [0.0, 0.0] }; AGENT_COUNT],
            positions: vec![Position { value: [0.0, 0.0] }; AGENT_COUNT],
            transforms: vec![default_transform(); AGENT_COUNT]
        };

        // Generate random positions and directions
        let mut rng = rand::thread_rng();

        for p in components.positions.iter_mut() {
            p.value = [
                rng.gen_range(0.0..2000.0),
                rng.gen_range(0.0..1000.0),
            ];
        }

        for d in components.directions.iter_mut() {
            d.direction = [
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
            ];
        }

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

        keep_on_screen_system(&self.components.positions, &mut self.components.directions, &self.display_size);

        caluclate_transform_system(&mut self.components.transforms, &self.components.positions);
    }

    pub fn on_keyboard(&mut self, _input: KeyboardInput) {}

    pub fn on_mouse_move(&mut self, _position: &PhysicalPosition<f64>) {}

    pub fn on_window_resize(&mut self, size: &PhysicalSize<u32>) {
        self.display_size = *size;

        self.perspective = perspective(
            self.display_size.width, 
            self.display_size.height
        );
    }
}