#!/usr/bin/env python3
"""Generate a 60 m Gazebo Classic spiral terrain, obstacles and previews.
Requires numpy, scipy, Pillow and matplotlib. No downloaded assets.
"""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/rough_terrain_matplotlib')
import numpy as np
from scipy.spatial import cKDTree
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource, ListedColormap
from matplotlib.patches import Circle, Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = Path(__file__).resolve().parent
N, WIDTH, HEIGHT = 257, 60.0, 4.0
x = np.linspace(-WIDTH / 2, WIDTH / 2, N)
y = x[::-1]
X, Y = np.meshgrid(x, y)


def smoothstep(t):
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


# Archimedean spiral: 1.5 turns, radius 23 -> 9 m, 3 m climb.
# A 3.6 m driving surface plus 1.6 m shoulders on each side.
t = np.linspace(0, 1, 3001)
theta = -np.pi / 2 + 3 * np.pi * t
radius = 23 - 14 * t
path = np.column_stack((radius * np.cos(theta), radius * np.sin(theta)))
distance, nearest = cKDTree(path).query(np.column_stack((X.ravel(), Y.ravel())))
distance, nearest = distance.reshape(X.shape), nearest.reshape(X.shape)
road_t = t[nearest]
road_weight = 1 - smoothstep((distance - 1.8) / 1.6)
road_height = 3 * smoothstep(road_t)
# Gentle longitudinal undulations in selected parts of the road.
road_bumps = (0.06 * np.sin(road_t * 90) *
              np.exp(-((road_t - 0.42) / 0.13)**4))
Z = road_weight * (road_height + road_bumps)

# Small hills, depressions and a rough-ground test zone beside the road.
def hill(cx, cy, amplitude, sx, sy):
    return amplitude * np.exp(-((X - cx) / sx)**2 - ((Y - cy) / sy)**2)

terrain = hill(-22, 19, 0.8, 3, 2.5) + hill(22, 19, 0.6, 2.7, 3)
terrain += hill(-23, -8, 0.45, 2.4, 3) + hill(5, -4, 0.3, 1.5, 1.8)
rough = np.exp(-((X + 4) / 4)**4 - ((Y + 5) / 2.5)**4)
terrain += rough * (0.18 + 0.07 * np.sin(3.5 * X) * np.cos(3 * Y))
# A shallow basin in a raised patch remains above the map base.
terrain += hill(20, -19, 0.4, 3.0, 3.0) - hill(20, -19, 0.22, 1, 1)
Z += np.maximum(terrain, 0) * (1 - road_weight)
# Level 2.5 m spawn pad at the world origin.
Z *= smoothstep((np.hypot(X, Y) - 2.5) / 0.8)
assert Z.min() >= -1e-8 and Z.max() <= HEIGHT
pixels = np.rint(np.clip(Z, 0, HEIGHT) / HEIGHT * 255).astype(np.uint8)
Image.fromarray(pixels).save(OUT / 'rough_terrain_heightmap.png')
Z = pixels.astype(float) / 255 * HEIGHT

# One static mesh for both collision and display: no heightmap renderer,
# terrain texture layers, normal maps or terrain shader generation.
vertices = np.stack((X, Y, Z), axis=-1).astype(np.float32)
a, b = vertices[:-1, :-1], vertices[:-1, 1:]
c, d = vertices[1:, :-1], vertices[1:, 1:]
# Image rows run toward -Y; these windings give upward-facing normals.
triangles = np.concatenate((np.stack((a, c, b), axis=-2).reshape(-1, 3, 3),
                            np.stack((b, c, d), axis=-2).reshape(-1, 3, 3)))
def write_stl(filename, faces):
    faces = np.asarray(faces, dtype=np.float32)
    normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    records = np.zeros(len(faces), dtype=[('normal', '<f4', (3,)),
                                        ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    records['normal'], records['vertices'] = normals, faces
    with (OUT / filename).open('wb') as mesh:
        mesh.write(b'Gazebo static mesh: collision and visual'.ljust(80, b'\0'))
        mesh.write(len(records).to_bytes(4, 'little'))
        mesh.write(records.tobytes())


write_stl('rough_terrain_mesh.stl', triangles)


def mesh_height(points):
    """Interpolate the same triangles used in the terrain collision mesh."""
    u = (points[:, 0] + WIDTH/2) / (WIDTH/(N-1))
    v = (WIDTH/2 - points[:, 1]) / (WIDTH/(N-1))
    j = np.clip(u.astype(int), 0, N-2)
    i = np.clip(v.astype(int), 0, N-2)
    u, v = u-j, v-i
    return np.where(u+v <= 1,
                    Z[i,j]*(1-u-v) + Z[i,j+1]*u + Z[i+1,j]*v,
                    Z[i+1,j+1]*(u+v-1) + Z[i,j+1]*(1-v) + Z[i+1,j]*(1-u))


# Merge every rail, post and curb into one static mesh/model, rather than
# creating hundreds of Gazebo entities. No texture layers or fence shadows.
fence_faces, fence_sides = [], []
box_indices = np.array([[0,2,1], [0,3,2], [4,5,6], [4,6,7],
                        [0,1,5], [0,5,4], [1,2,6], [1,6,5],
                        [2,3,7], [2,7,6], [3,0,4], [3,4,7]])


def fence_beam(p, q, width, thickness):
    direction = q[:2] - p[:2]
    side = np.array([-direction[1], direction[0]]) / np.linalg.norm(direction) * width/2
    footprint = np.array([p[:2]-side, q[:2]-side, q[:2]+side, p[:2]+side])
    center_z = np.array([p[2], q[2], q[2], p[2]])
    bottom = np.column_stack((footprint, center_z-thickness/2))
    top = np.column_stack((footprint, center_z+thickness/2))
    fence_faces.extend(np.vstack((bottom, top))[box_indices])


arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
fence_arc = np.linspace(0.75, arc[-1]-0.75, int(arc[-1]/1.25)+1)
ft = np.interp(fence_arc, arc, t)
angle = -np.pi/2 + 3*np.pi*ft
rad = 23 - 14*ft
centerline = np.column_stack((rad*np.cos(angle), rad*np.sin(angle)))
tangent = np.column_stack((-14*np.cos(angle)-3*np.pi*rad*np.sin(angle),
                           -14*np.sin(angle)+3*np.pi*rad*np.cos(angle)))
normal = np.column_stack((-tangent[:,1], tangent[:,0]))
normal /= np.linalg.norm(normal, axis=1)[:,None]
for sign in [-1, 1]:
    edge = centerline + sign*2.05*normal
    ground = mesh_height(edge)
    fence_sides.append((edge, ground))
    for k in range(len(edge)-1):
        # Solid lower wall from 5 cm below ground to 45 cm above it;
        # retain the upper rail and posts, with matching visual/collision mesh.
        for offset, thick in [(0.20, 0.50), (0.85, 0.10)]:
            p = np.r_[edge[k], ground[k]+offset]
            q = np.r_[edge[k+1], ground[k+1]+offset]
            # Tiny overlap closes joints around the curved track.
            along = (q-p) / np.linalg.norm(q-p) * 0.015
            fence_beam(p-along, q+along, 0.12, thick)
    for k in range(0, len(edge), 2):
        # Short horizontal footprint, tall vertical prism = one fence post.
        p = np.r_[edge[k]-np.array([0.06,0]), ground[k]+0.425]
        q = np.r_[edge[k]+np.array([0.06,0]), ground[k]+0.425]
        fence_beam(p, q, 0.12, 0.95)
fence_faces = np.asarray(fence_faces, dtype=np.float32)
write_stl('rough_terrain_fence.stl', fence_faces)
fence_xml = '''<model name="spiral_fence"><static>true</static><link name="fence">
      <collision name="collision"><geometry><mesh>
        <uri>file://worlds/rough_terrain_fence.stl</uri>
      </mesh></geometry></collision>
      <visual name="visual"><cast_shadows>false</cast_shadows><geometry><mesh>
        <uri>file://worlds/rough_terrain_fence.stl</uri>
      </mesh></geometry><material><lighting>true</lighting><script>
        <uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/DarkGrey</name>
      </script></material></visual>
    </link></model>'''

# kind, x, y, footprint dimensions (radius for cylinders), height, yaw.
obstacles = [
    ('cylinder', -25, -15, (0.65,), 1.6, 0),
    ('cylinder', -24, 9, (0.7,), 2.0, 0),
    ('cylinder', -17, 24, (0.8,), 1.8, 0),
    ('cylinder', 17, 25, (0.65,), 2.2, 0),
    ('cylinder', 25, 4, (0.8,), 1.6, 0),
    ('cylinder', 4, 2, (0.55,), 1.4, 0),
    ('box', -25, 0, (2.0, 1.3), 1.3, 0.3),
    ('box', -8, 25, (2.2, 1.4), 1.7, -0.3),
    ('box', 25, -12, (2.4, 1.4), 1.2, 0.5),
    ('box', 10, -26, (2.5, 1.5), 1.5, -0.4),
    ('box', -5, 3, (1.8, 1.2), 1.1, 0.4),
    ('box', 1, 6, (2.0, 1.2), 1.5, 0),
]
# Narrow obstacles near alternate road edges leave a passing lane.
for k, fraction in enumerate([0.22, 0.48, 0.73]):
    i = int(fraction * (len(t) - 1))
    tangent = path[i + 1] - path[i - 1]
    normal = np.array([-tangent[1], tangent[0]]) / np.linalg.norm(tangent)
    px, py = path[i] + normal * (1.0 if k % 2 == 0 else -1.0)
    obstacles.append(('cylinder', px, py, (0.32,), 0.85, 0))


def support_height(px, py, dims, kind, yaw):
    """Embed base 3 cm below highest terrain sample under the footprint."""
    dx, dy = X - px, Y - py
    u, v = np.cos(yaw) * dx + np.sin(yaw) * dy, -np.sin(yaw) * dx + np.cos(yaw) * dy
    if kind == 'cylinder':
        mask = u*u + v*v <= dims[0]**2
    else:
        mask = (abs(u) <= dims[0]/2) & (abs(v) <= dims[1]/2)
    return float(Z[mask].max()) - 0.03


obstacle_xml, placed = [], []
for i, (kind, px, py, dims, height, yaw) in enumerate(obstacles):
    base = support_height(px, py, dims, kind, yaw)
    if kind == 'cylinder':
        shape = f'<cylinder><radius>{dims[0]}</radius><length>{height}</length></cylinder>'
        material_name = 'Gazebo/Orange'
    else:
        shape = f'<box><size>{dims[0]} {dims[1]} {height}</size></box>'
        material_name = 'Gazebo/Blue'
    obstacle_xml.append(f'''<model name="{kind}_{i:02d}">
      <static>true</static><pose>{px} {py} {base + height/2} 0 0 {yaw}</pose>
      <link name="body">
        <collision name="collision"><geometry>{shape}</geometry></collision>
        <visual name="visual"><cast_shadows>true</cast_shadows><geometry>{shape}</geometry>
          <material><lighting>true</lighting><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{material_name}</name></script></material>
        </visual>
      </link>
    </model>''')
    placed.append((kind, px, py, dims, height, yaw, base))

# Resolve through GAZEBO_RESOURCE_PATH, which the package launch configures.
# Keep the installed world independent of the source checkout's location.
uri = 'file://worlds/rough_terrain_mesh.stl'
geometry = f'<geometry><mesh><uri>{uri}</uri></mesh></geometry>'
visual_geometry = geometry
world = f'''<?xml version="1.0"?>
<sdf version="1.7">
  <world name="rough_terrain">
    <gravity>0 0 -9.81</gravity>
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene><shadows>true</shadows><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background></scene>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular><direction>-0.5 0.1 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>
    <model name="rough_terrain">
      <static>true</static><link name="terrain">
        <collision name="collision">{geometry}
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>{visual_geometry}
          <material><lighting>true</lighting><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
    {''.join(obstacle_xml)}
    {fence_xml}
    <gui><camera name="overview"><pose>-44 -48 50 0 0.65 0.83</pose></camera></gui>
  </world>
</sdf>
'''
(OUT / 'rough_terrain.world').write_text(world)

# Use the same quantized heightfield and obstacle geometry in the preview.
fig = plt.figure(figsize=(17, 9), facecolor='#f5f7fa')
ax = fig.add_subplot(121, projection='3d', computed_zorder=False)
step = 5
xx, yy, zz = X[::step, ::step], Y[::step, ::step], Z[::step, ::step]
earth = ListedColormap(plt.cm.gist_earth(np.linspace(0.35, 0.9, 256)))
colors = LightSource(315, 45).shade(zz, cmap=earth, vert_exag=2,
    dx=WIDTH/(N-1)*step, dy=WIDTH/(N-1)*step)
ax.plot_surface(xx, yy, zz, facecolors=colors, rstride=1, cstride=1,
                linewidth=0, antialiased=True, shade=False, zorder=1)
ax2 = fig.add_subplot(122)
im = ax2.imshow(Z, extent=(-30, 30, -30, 30), origin='upper', cmap=earth, vmin=0, vmax=3.2)
ax.add_collection3d(Poly3DCollection(fence_faces, facecolors='#424a52', linewidths=0, zorder=3))
for i, (edge, ground) in enumerate(fence_sides):
    ax2.plot(edge[:,0], edge[:,1], color='#303840', lw=1.2,
             label='Fence (both sides)' if i == 0 else None)
for kind, px, py, dims, height, yaw, base in placed:
    color = '#e65a21' if kind == 'cylinder' else '#30699c'
    if kind == 'cylinder':
        a = np.linspace(0, 2*np.pi, 25)
        cx, cy = px + dims[0]*np.cos(a), py + dims[0]*np.sin(a)
        ax.plot_surface(np.array([cx, cx]), np.array([cy, cy]),
                        np.array([a*0+base, a*0+base+height]), color=color, shade=True, zorder=4)
        ax.add_collection3d(Poly3DCollection([list(zip(cx, cy, a*0+base+height))], facecolors=color, zorder=4))
        ax2.add_patch(Circle((px, py), dims[0], color=color))
    else:
        corners = np.array([[-1,-1],[1,-1],[1,1],[-1,1]]) * np.array(dims)/2
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        corners = corners @ rot.T + [px, py]
        bottom = np.column_stack((corners, np.full(4, base)))
        top = bottom + [0,0,height]
        faces = [top] + [[bottom[j], bottom[(j+1)%4], top[(j+1)%4], top[j]] for j in range(4)]
        ax.add_collection3d(Poly3DCollection(faces, facecolors=color, edgecolors='#254766', linewidths=0.4, zorder=4))
        ax2.add_patch(Polygon(corners, color=color))
ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Height (m)', zlim=(0,5),
       title='3D terrain + physical obstacles | height exaggerated 3x')
ax.set_box_aspect((60,60,15))
ax.view_init(elev=51, azim=-125)
ax.tick_params(labelsize=8, pad=0)
ax2.plot(path[:,0], path[:,1], '--', color='white', lw=1.0, alpha=0.9, label='Spiral centreline')
for px, py, label in [(0,0,'Flat pad'), (0,-23,'Spawn / entry: 0 m'), (0,9,'Finish: 3 m'),
                       (-22,19,'Small hill'), (-4,-5,'Uneven ground')]:
    ax2.plot(px, py, 'o', color='#c92040', ms=4)
    ax2.annotate(label, (px,py), xytext=(5,7), textcoords='offset points', fontsize=8,
                 bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=2))
ax2.set(xlabel='X (m)', ylabel='Y (m)', title='Top view | orange: cylinders, blue: boxes')
ax2.legend(loc='upper left', fontsize=8)
fig.colorbar(im, ax=ax2, shrink=0.65, label='Ground elevation (m)', pad=0.02)
fig.suptitle('60 x 60 m | spiral ramp with fences | 15 obstacles | uneven roads', fontsize=16, y=0.96)
fig.subplots_adjust(left=0.055, right=0.96, bottom=0.08, top=0.88, wspace=0.15)
fig.savefig(OUT / 'rough_terrain_preview.png', dpi=160, bbox_inches='tight', pad_inches=0.3)
plt.close(fig)
length = np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
assert Z[N//2,N//2] == 0
print(f'World: {OUT / "rough_terrain.world"}')
print(f'Preview: {OUT / "rough_terrain_preview.png"}')
print(f'Spiral length {length:.1f} m; terrain max {Z.max():.3f} m; {len(placed)} obstacles; spawn z=0')
print(f'Fence: two open-ended sides; {len(fence_faces)} triangles; 0.45 m lower wall, 0.9 m total height; 4 m clear width')
