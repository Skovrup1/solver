package main

import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:time"

import "vendor:raylib"

Vec2 :: [2]f32
Vec3 :: [3]f32

dot :: linalg.dot

Color :: raylib.Color

SCREEN_WIDTH :: 800
SCREEN_HEIGHT :: 600

TARGET_FPS :: 240

UPDATE_RATE :: 60 // Hz
UPDATE_TIME :: time.Second / time.Duration(UPDATE_RATE) // ns

GRAVITY :: 64
DAMPING :: 0.95

particles: #soa[dynamic]Particle
dynamics: #soa[dynamic]Dynamics2D

Dynamics2D :: struct {
	inv_mass:     f32, // 1 / mass
	damping:      f32,
	position:     Vec2,
	velocity:     Vec2,
	acceleration: Vec2,
	gravity:      Vec2,
}

get_inv_mass :: proc(mass: f32) -> f32 {
	return 1 / mass
}

set_mass :: proc(dyn: ^Dynamics2D, mass: f32) {
	dyn.inv_mass = 1 / mass
}

Particle :: struct {
	size:     Vec2,
	dynamics: int, // index into dynamics
	color:    Color,
}

make_particle :: proc(
	size: Vec2,
	position: Vec2 = {},
	velocity: Vec2 = {},
	acceleration: Vec2 = {},
	gravity: Vec2 = {0, GRAVITY},
	inv_mass: f32 = 0,
	damping: f32 = 0.95,
	color: Color = raylib.GRAY,
) -> Particle {
	dyn := Dynamics2D{inv_mass, damping, position, velocity, acceleration, gravity}
	index := len(dynamics)
	append(&dynamics, dyn)

	return Particle{size, index, color}
}

draw_particles :: proc() {
	for particle in particles {
		dyn := dynamics[particle.dynamics]
		raylib.DrawRectangleV(dyn.position, particle.size, particle.color)
	}
}

integrate :: proc(time_step: f32) {
	for &dyn in dynamics {
		dyn.position += dyn.velocity * time_step

		force_accumulated := dyn.gravity
		dyn.acceleration = force_accumulated * dyn.inv_mass

		dyn.velocity += dyn.acceleration * time_step
		dyn.velocity *= math.pow(dyn.damping, time_step)
	}
}

update :: proc(time_step: f32) {
	integrate(time_step)
}

draw :: proc() {
	raylib.BeginDrawing()

	raylib.ClearBackground(raylib.RAYWHITE)

	draw_particles()

	raylib.EndDrawing()
}

main :: proc() {
	raylib.SetTraceLogLevel(raylib.TraceLogLevel.WARNING)
	raylib.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics solver")
	defer raylib.CloseWindow()

	raylib.SetTargetFPS(TARGET_FPS)

	//floor_size := Vec2{SCREEN_WIDTH, 100}
	//floor_pos := Vec2{0, 500}
	//append(&particles, make_particle(floor_size, floor_pos))

	for i in 0 ..< 32 {
		particle_size := Vec2{4, 4}
		particle_pos := Vec2{SCREEN_WIDTH / 2.0, 500}
		displace := f32(i) * 1.5
		displace = i % 2 == 0 ? -displace : displace
		particle_acc := Vec2{displace, -256}
		append(
			&particles,
			make_particle(
				particle_size,
				particle_pos,
				particle_acc,
				inv_mass = get_inv_mass(1),
				color = raylib.MAROON,
			),
		)
	}

	accumulator: time.Duration = 0
	for !raylib.WindowShouldClose() {
		accumulator += time.Duration(f64(raylib.GetFrameTime()) * f64(time.Second))

		for accumulator >= UPDATE_TIME {
			update_time_seconds := cast(f32)time.duration_seconds(UPDATE_TIME)
			update(update_time_seconds)
			accumulator -= UPDATE_TIME
		}

		draw()
	}
}
