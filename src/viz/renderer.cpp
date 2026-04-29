#include "renderer.h"

#include "scene_snapshot.h"

#include <glad/gl.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

namespace robosim::viz {

namespace {

// Mesh: VAO + VBO + EBO with position(vec3) + normal(vec3) interleaved.
struct mesh {
  GLuint vao = 0;
  GLuint vbo = 0;
  GLuint ebo = 0;
  GLsizei element_count = 0;
  GLenum primitive_mode = GL_TRIANGLES;
};

constexpr int cylinder_segments = 32;
constexpr float pi_f = 3.14159265358979F;

[[nodiscard]] mesh build_indexed_mesh(const std::vector<float>& verts,
                                      const std::vector<unsigned int>& idx) {
  mesh m;
  glGenVertexArrays(1, &m.vao);
  glGenBuffers(1, &m.vbo);
  glGenBuffers(1, &m.ebo);
  glBindVertexArray(m.vao);
  glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
               verts.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(idx.size() * sizeof(unsigned int)),
               idx.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                        static_cast<GLsizei>(6 * sizeof(float)), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(
      1, 3, GL_FLOAT, GL_FALSE,
      static_cast<GLsizei>(6 * sizeof(float)),
      reinterpret_cast<void*>(3 * sizeof(float)));  // NOLINT
  glBindVertexArray(0);
  m.element_count = static_cast<GLsizei>(idx.size());
  m.primitive_mode = GL_TRIANGLES;
  return m;
}

[[nodiscard]] mesh build_line_mesh(const std::vector<float>& verts) {
  mesh m;
  glGenVertexArrays(1, &m.vao);
  glGenBuffers(1, &m.vbo);
  glBindVertexArray(m.vao);
  glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
               verts.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                        static_cast<GLsizei>(6 * sizeof(float)), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(
      1, 3, GL_FLOAT, GL_FALSE,
      static_cast<GLsizei>(6 * sizeof(float)),
      reinterpret_cast<void*>(3 * sizeof(float)));  // NOLINT
  glBindVertexArray(0);
  m.element_count = static_cast<GLsizei>(verts.size() / 6);
  m.primitive_mode = GL_LINES;
  return m;
}

// Unit cylinder along +Z, radius 1, length 1, base at the local
// origin (proximal cap at z = 0, distal cap at z = 1). Per TEST_PLAN
// conventions #8 / #9, the cylinder/arrow primitive is anchored at
// its base, not centered.
[[nodiscard]] mesh build_unit_cylinder() {
  std::vector<float> v;
  std::vector<unsigned int> i;

  // Side ring vertices.
  for (int s = 0; s < cylinder_segments; ++s) {
    const float a =
        2.0F * pi_f * static_cast<float>(s) /
        static_cast<float>(cylinder_segments);
    const float cx = std::cos(a);
    const float cy = std::sin(a);
    v.insert(v.end(), {cx, cy, 0.0F, cx, cy, 0.0F});
    v.insert(v.end(), {cx, cy, 1.0F, cx, cy, 0.0F});
  }
  for (int s = 0; s < cylinder_segments; ++s) {
    const unsigned int b0 = static_cast<unsigned int>(s * 2);
    const unsigned int t0 = b0 + 1;
    const unsigned int b1 =
        static_cast<unsigned int>(((s + 1) % cylinder_segments) * 2);
    const unsigned int t1 = b1 + 1;
    i.insert(i.end(), {b0, b1, t1, b0, t1, t0});
  }

  // Proximal cap (z = 0).
  const unsigned int bottom_center =
      static_cast<unsigned int>(v.size() / 6);
  v.insert(v.end(), {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -1.0F});
  for (int s = 0; s < cylinder_segments; ++s) {
    const float a =
        2.0F * pi_f * static_cast<float>(s) /
        static_cast<float>(cylinder_segments);
    v.insert(v.end(),
             {std::cos(a), std::sin(a), 0.0F, 0.0F, 0.0F, -1.0F});
  }
  for (int s = 0; s < cylinder_segments; ++s) {
    const unsigned int a = bottom_center + 1 + static_cast<unsigned int>(s);
    const unsigned int b =
        bottom_center + 1 +
        static_cast<unsigned int>((s + 1) % cylinder_segments);
    i.insert(i.end(), {bottom_center, b, a});
  }

  // Distal cap (z = 1).
  const unsigned int top_center = static_cast<unsigned int>(v.size() / 6);
  v.insert(v.end(), {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F});
  for (int s = 0; s < cylinder_segments; ++s) {
    const float a =
        2.0F * pi_f * static_cast<float>(s) /
        static_cast<float>(cylinder_segments);
    v.insert(v.end(),
             {std::cos(a), std::sin(a), 1.0F, 0.0F, 0.0F, 1.0F});
  }
  for (int s = 0; s < cylinder_segments; ++s) {
    const unsigned int a = top_center + 1 + static_cast<unsigned int>(s);
    const unsigned int b =
        top_center + 1 +
        static_cast<unsigned int>((s + 1) % cylinder_segments);
    i.insert(i.end(), {top_center, a, b});
  }
  return build_indexed_mesh(v, i);
}

[[nodiscard]] mesh build_unit_box() {
  const std::vector<float> v = {
      -1, -1, -1, -1, 0,  0,  -1, 1,  -1, -1, 0,  0,  -1, 1,  1,  -1,
      0,  0,  -1, -1, 1,  -1, 0,  0,  1,  -1, -1, 1,  0,  0,  1,  -1,
      1,  1,  0,  0,  1,  1,  1,  1,  0,  0,  1,  1,  -1, 1,  0,  0,
      -1, -1, -1, 0,  -1, 0,  -1, -1, 1,  0,  -1, 0,  1,  -1, 1,  0,
      -1, 0,  1,  -1, -1, 0,  -1, 0,  -1, 1,  -1, 0,  1,  0,  1,  1,
      -1, 0,  1,  0,  1,  1,  1,  0,  1,  0,  -1, 1,  1,  0,  1,  0,
      -1, -1, -1, 0,  0,  -1, 1,  -1, -1, 0,  0,  -1, 1,  1,  -1, 0,
      0,  -1, -1, 1,  -1, 0,  0,  -1, -1, -1, 1,  0,  0,  1,  -1, 1,
      1,  0,  0,  1,  1,  1,  1,  0,  0,  1,  1,  -1, 1,  0,  0,  1,
  };
  std::vector<unsigned int> i(36);
  for (unsigned int f = 0; f < 6; ++f) {
    const unsigned int b = f * 4;
    const unsigned int o = f * 6;
    i[o + 0] = b + 0;
    i[o + 1] = b + 1;
    i[o + 2] = b + 2;
    i[o + 3] = b + 0;
    i[o + 4] = b + 2;
    i[o + 5] = b + 3;
  }
  return build_indexed_mesh(v, i);
}

[[nodiscard]] mesh build_unit_sphere() {
  constexpr int rings = 16;
  constexpr int sectors = 32;
  std::vector<float> v;
  std::vector<unsigned int> i;
  for (int r = 0; r <= rings; ++r) {
    const float vt = static_cast<float>(r) / static_cast<float>(rings);
    const float phi = pi_f * vt;
    for (int s = 0; s <= sectors; ++s) {
      const float u = static_cast<float>(s) / static_cast<float>(sectors);
      const float theta = 2.0F * pi_f * u;
      const float x = std::sin(phi) * std::cos(theta);
      const float y = std::cos(phi);
      const float z = std::sin(phi) * std::sin(theta);
      v.insert(v.end(), {x, y, z, x, y, z});
    }
  }
  for (int r = 0; r < rings; ++r) {
    for (int s = 0; s < sectors; ++s) {
      const unsigned int a = static_cast<unsigned int>(r * (sectors + 1) + s);
      const unsigned int b = a + static_cast<unsigned int>(sectors + 1);
      i.insert(i.end(), {a, b, a + 1, b, b + 1, a + 1});
    }
  }
  return build_indexed_mesh(v, i);
}

[[nodiscard]] mesh build_grid(int half_extent_cells) {
  std::vector<float> v;
  for (int i = -half_extent_cells; i <= half_extent_cells; ++i) {
    const float c = static_cast<float>(i);
    const float e = static_cast<float>(half_extent_cells);
    v.insert(v.end(), {-e, 0, c, 0, 1, 0, e, 0, c, 0, 1, 0});
    v.insert(v.end(), {c, 0, -e, 0, 1, 0, c, 0, e, 0, 1, 0});
  }
  return build_line_mesh(v);
}

void destroy_mesh(mesh& m) {
  if (m.ebo != 0) glDeleteBuffers(1, &m.ebo);
  if (m.vbo != 0) glDeleteBuffers(1, &m.vbo);
  if (m.vao != 0) glDeleteVertexArrays(1, &m.vao);
  m = {};
}

constexpr const char* mesh_vertex_src = R"(
#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
uniform mat4 u_mvp;
uniform mat4 u_model;
out vec3 v_world_normal;
void main() {
  v_world_normal = mat3(u_model) * a_normal;
  gl_Position = u_mvp * vec4(a_position, 1.0);
}
)";

constexpr const char* mesh_fragment_src = R"(
#version 330 core
in vec3 v_world_normal;
out vec4 frag_color;
uniform vec3 u_color;
uniform vec3 u_light_dir;
uniform float u_ambient;
void main() {
  vec3 n = normalize(v_world_normal);
  float diffuse = max(dot(n, normalize(u_light_dir)), 0.0);
  vec3 lit = u_color * (u_ambient + (1.0 - u_ambient) * diffuse);
  frag_color = vec4(lit, 1.0);
}
)";

constexpr const char* line_vertex_src = R"(
#version 330 core
layout(location = 0) in vec3 a_position;
uniform mat4 u_mvp;
void main() { gl_Position = u_mvp * vec4(a_position, 1.0); }
)";

constexpr const char* line_fragment_src = R"(
#version 330 core
uniform vec3 u_color;
out vec4 frag_color;
void main() { frag_color = vec4(u_color, 1.0); }
)";

[[nodiscard]] GLuint compile_shader(GLenum type, const char* src) {
  const GLuint s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (ok == 0) {
    char log[1024];
    glGetShaderInfoLog(s, sizeof(log), nullptr, log);
    std::fprintf(stderr, "robosim-viz: shader compile failed: %s\n", log);
  }
  return s;
}

[[nodiscard]] GLuint link_program(GLuint vs, GLuint fs) {
  const GLuint p = glCreateProgram();
  glAttachShader(p, vs);
  glAttachShader(p, fs);
  glLinkProgram(p);
  GLint ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (ok == 0) {
    char log[1024];
    glGetProgramInfoLog(p, sizeof(log), nullptr, log);
    std::fprintf(stderr, "robosim-viz: program link failed: %s\n", log);
  }
  glDeleteShader(vs);
  glDeleteShader(fs);
  return p;
}

using mat4d = std::array<std::array<double, 4>, 4>;

[[nodiscard]] mat4d matmul(const mat4d& a, const mat4d& b) {
  mat4d r{};
  for (int c = 0; c < 4; ++c) {
    for (int row = 0; row < 4; ++row) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k) s += a[k][row] * b[c][k];
      r[c][row] = s;
    }
  }
  return r;
}

[[nodiscard]] std::array<std::array<float, 4>, 4> to_float(const mat4d& m) {
  std::array<std::array<float, 4>, 4> r{};
  for (int c = 0; c < 4; ++c) {
    for (int row = 0; row < 4; ++row) {
      r[c][row] = static_cast<float>(m[c][row]);
    }
  }
  return r;
}

[[nodiscard]] mat4d apply_scale(const mat4d& m, double sx, double sy,
                                double sz) {
  auto r = m;
  for (int row = 0; row < 4; ++row) {
    r[0][row] *= sx;
    r[1][row] *= sy;
    r[2][row] *= sz;
  }
  return r;
}

}  // namespace

struct renderer::impl {
  GLuint mesh_program = 0;
  GLint mesh_u_mvp = -1;
  GLint mesh_u_model = -1;
  GLint mesh_u_color = -1;
  GLint mesh_u_light_dir = -1;
  GLint mesh_u_ambient = -1;

  GLuint line_program = 0;
  GLint line_u_mvp = -1;
  GLint line_u_color = -1;

  mesh cylinder_mesh;
  mesh box_mesh;
  mesh sphere_mesh;
  mesh grid_mesh;
};

renderer::renderer() : impl_(std::make_unique<impl>()) {
  GLuint vs = compile_shader(GL_VERTEX_SHADER, mesh_vertex_src);
  GLuint fs = compile_shader(GL_FRAGMENT_SHADER, mesh_fragment_src);
  impl_->mesh_program = link_program(vs, fs);
  impl_->mesh_u_mvp = glGetUniformLocation(impl_->mesh_program, "u_mvp");
  impl_->mesh_u_model = glGetUniformLocation(impl_->mesh_program, "u_model");
  impl_->mesh_u_color = glGetUniformLocation(impl_->mesh_program, "u_color");
  impl_->mesh_u_light_dir =
      glGetUniformLocation(impl_->mesh_program, "u_light_dir");
  impl_->mesh_u_ambient =
      glGetUniformLocation(impl_->mesh_program, "u_ambient");

  vs = compile_shader(GL_VERTEX_SHADER, line_vertex_src);
  fs = compile_shader(GL_FRAGMENT_SHADER, line_fragment_src);
  impl_->line_program = link_program(vs, fs);
  impl_->line_u_mvp = glGetUniformLocation(impl_->line_program, "u_mvp");
  impl_->line_u_color = glGetUniformLocation(impl_->line_program, "u_color");

  impl_->cylinder_mesh = build_unit_cylinder();
  impl_->box_mesh = build_unit_box();
  impl_->sphere_mesh = build_unit_sphere();
  impl_->grid_mesh = build_grid(5);
}

renderer::~renderer() {
  if (impl_) {
    destroy_mesh(impl_->cylinder_mesh);
    destroy_mesh(impl_->box_mesh);
    destroy_mesh(impl_->sphere_mesh);
    destroy_mesh(impl_->grid_mesh);
    if (impl_->mesh_program != 0) glDeleteProgram(impl_->mesh_program);
    if (impl_->line_program != 0) glDeleteProgram(impl_->line_program);
  }
}

void renderer::draw(
    const scene_snapshot& s, const mat4d& view,
    const mat4d& projection, std::array<int, 2> framebuffer_size) {
  glViewport(0, 0, framebuffer_size[0], framebuffer_size[1]);
  glClearColor(0.10F, 0.11F, 0.12F, 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);

  const mat4d vp = matmul(projection, view);
  const auto vp_f = to_float(vp);

  // Grid (lines).
  glUseProgram(impl_->line_program);
  glUniformMatrix4fv(impl_->line_u_mvp, 1, GL_FALSE, &vp_f[0][0]);
  glUniform3f(impl_->line_u_color, 0.30F, 0.30F, 0.32F);
  glBindVertexArray(impl_->grid_mesh.vao);
  glDrawArrays(GL_LINES, 0, impl_->grid_mesh.element_count);

  // Mesh.
  glUseProgram(impl_->mesh_program);
  glUniform3f(impl_->mesh_u_light_dir, 0.4F, 0.8F, 0.5F);
  glUniform1f(impl_->mesh_u_ambient, 0.25F);

  for (std::size_t i = 0; i < s.nodes.size(); ++i) {
    const auto& node = s.nodes[i];
    if (!node.shape) continue;

    mat4d model = node.world_from_local.m;
    const mesh* m_ptr = nullptr;
    switch (node.shape->kind) {
      case primitive_kind::cylinder:
      case primitive_kind::arrow:
        model = apply_scale(model, node.shape->radius_m,
                            node.shape->radius_m, node.shape->length_m);
        m_ptr = &impl_->cylinder_mesh;
        break;
      case primitive_kind::sphere:
        model = apply_scale(model, node.shape->radius_m,
                            node.shape->radius_m, node.shape->radius_m);
        m_ptr = &impl_->sphere_mesh;
        break;
      case primitive_kind::box:
        model = apply_scale(model, node.shape->half_extent_x_m,
                            node.shape->half_extent_y_m,
                            node.shape->half_extent_z_m);
        m_ptr = &impl_->box_mesh;
        break;
    }
    if (m_ptr == nullptr) continue;

    const mat4d mvp = matmul(projection, matmul(view, model));
    const auto mvp_f = to_float(mvp);
    const auto model_f = to_float(model);
    glUniformMatrix4fv(impl_->mesh_u_mvp, 1, GL_FALSE, &mvp_f[0][0]);
    glUniformMatrix4fv(impl_->mesh_u_model, 1, GL_FALSE, &model_f[0][0]);

    const bool selected =
        s.selected_index.has_value() && *s.selected_index == i;
    const bool is_joint = node.kind == node_kind::joint;
    float r = is_joint ? 0.95F : 0.55F;
    float g = is_joint ? 0.65F : 0.70F;
    float b = is_joint ? 0.20F : 0.85F;
    if (selected) {
      r = 1.0F;
      g = 0.85F;
      b = 0.20F;
    }
    glUniform3f(impl_->mesh_u_color, r, g, b);

    glBindVertexArray(m_ptr->vao);
    glDrawElements(GL_TRIANGLES, m_ptr->element_count, GL_UNSIGNED_INT,
                   nullptr);
  }
  glBindVertexArray(0);
}

}  // namespace robosim::viz
