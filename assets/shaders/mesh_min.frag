#version 410 core
in vec3 v_Normal;
out vec4 FragColor;

uniform vec3 u_LightDir;
uniform vec3 u_Color;

void main() {
    float d = max(dot(normalize(v_Normal), normalize(-u_LightDir)), 0.0);
    vec3 col = (0.5 + 0.5 * d) * u_Color;
    FragColor = vec4(col, 1.0);
    // FragColor = vec4(normalize(v_Normal) * 0.5 + 0.5, 1.0);

}
