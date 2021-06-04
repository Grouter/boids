use vecmath::{Matrix4, Vector2, Vector3};

#[derive(Clone, Copy)]
pub struct Vertex {
    pub position: Vector2<f32>,
    pub color: Vector3<f32>,
}
implement_vertex!(Vertex, position, color);

#[derive(Clone, Copy)]
pub struct Transform {
    pub transform: Matrix4<f32>
}
implement_vertex!(Transform, transform);

#[derive(Clone, Copy)]
pub struct Forward {
    pub direction: Vector2<f32>
}

#[derive(Clone, Copy)]
pub struct Position {
    pub value: Vector2<f32>
}