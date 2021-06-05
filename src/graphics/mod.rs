use std::fs;

use cgmath::conv::array4x4;
use glium::index::PrimitiveType;
use glium::{Display, IndexBuffer, Program, VertexBuffer};
use glium::glutin::ContextBuilder;
use glium::glutin::dpi::PhysicalSize;
use glium::glutin::event_loop::EventLoop;
use glium::glutin::window::WindowBuilder;

use crate::data::{Transform, Vertex};

pub struct Mesh {
    pub v_buffer: VertexBuffer<Vertex>,
    pub i_buffer: IndexBuffer<u16>,
}

pub fn create_agent_shape(size: f32, color: [f32; 3]) -> ([Vertex; 3], [u16; 3]) {
    let size_h = size / 2.0;

    // Create a simple square
    let vertices = [
        Vertex { position: [-size_h,  size_h], color },
        Vertex { position: [ size_h + 5.0, 0.0], color },
        Vertex { position: [-size_h, -size_h], color },
    ];

    let indices = [
        0, 2, 1,
    ];

    (vertices, indices)
}

pub fn create_mesh(display: &Display, vertices: &[Vertex], indices: &[u16]) -> Mesh {
    let v_buffer = VertexBuffer::new(
        display,
        vertices
    ).expect("Error creating vertex buffer");

    let i_buffer = IndexBuffer::new(
        display,
        PrimitiveType::TrianglesList,
        indices
    ).expect("Error creating index buffer");

    Mesh { v_buffer, i_buffer }
}

pub fn create_display(event_loop: &EventLoop<()>, w: u32, h: u32) -> Display {
    let display = Display::new(
        WindowBuilder::new()
            .with_title("Boids")
            .with_maximized(true),
        ContextBuilder::new(),
        &event_loop
    ).expect("Could not create display");

    display.gl_window().resize(PhysicalSize {
        width: w,
        height: h
    });

    display
}

pub fn load_program(display: &Display, vertex_shader: &str, fragment_shader: &str) -> Program {
    let vertex_source = fs::read_to_string(vertex_shader)
        .expect("Error while loading vertex shader");

    let fragment_source = fs::read_to_string(fragment_shader)
        .expect("Error while loading fragment shader");

    Program::from_source(
        display,
        vertex_source.as_str(),
        fragment_source.as_str(),
        None
    ).expect("Error in shader program compilation")
}

pub fn perspective(display_w: u32, display_h: u32) -> [[f32; 4]; 4] {
    const Z_NEAR: f32 = -1.0;
    const Z_FAR: f32 = 1.0;

    let ortho = cgmath::ortho::<f32>(
        0.0, 
        display_w as f32, 
        display_h as f32, 
        0.0, 
        Z_NEAR, 
        Z_FAR
    );

    array4x4(ortho)
}

pub fn default_transform() -> Transform {
    Transform { 
        transform: [
            [1.0, 0.0 ,0.0, 0.0],
            [0.0, 1.0 ,0.0, 0.0],
            [0.0, 0.0 ,1.0, 0.0],
            [0.0, 0.0 ,0.0, 1.0],
        ]
    }
}