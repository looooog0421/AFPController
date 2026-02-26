import coacd
import trimesh
import numpy as np

filename = "/home/lgx/Project/AFP/src/afp_mjc/env/90degree_model/meshes/90degree_model.STL"
mesh = trimesh.load(filename)
print(f"原始模型加载完成: {filename}")

coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)

# 2. 运行 CoACD
print("正在进行凸分解计算，请稍候...")
parts = coacd.run_coacd(
    coacd_mesh, 
    threshold=0.01  # 阈值越小，精度越高，生成的块越多
)
print(f"分解完成，共生成 {len(parts)} 个凸块")


# 创建一个场景管理器
scene = trimesh.Scene()

# 原始模型可以选择以线框模式加入，作为参考（可选）
mesh.visual.face_colors = [200, 200, 200, 50] # 设为极淡的灰色
# scene.add_geometry(mesh)

# 遍历分解后的结果，转为 Mesh 对象并加入场景
for verts, faces in parts:
    sub_mesh = trimesh.Trimesh(verts, faces)
    # 生成随机颜色
    random_color = np.append(np.random.randint(0, 255, 3), 200)
    sub_mesh.visual.face_colors = random_color
    scene.add_geometry(sub_mesh)

print("正在打开可视化窗口...")
scene.show()

# 4. 保存
# save_confirm = input("是否保存分解后的模型？(y/n): ")
# if save_confirm.lower() == 'y':
#     for i, (verts, faces) in enumerate(parts):
#         sub_mesh = trimesh.Trimesh(verts, faces)
#         sub_mesh.export(f"/home/lgx/Project/AFP/src/afp_mjc/env/90degree_model/meshes/90degree_model_part_{i}.STL")
#     print("保存完成。")

# ==========================================
# 3. 生成 MuJoCo XML 内容
# ==========================================

obj_name = "90degree_model"
part_filenames = [f"{obj_name}_part_{i}.STL" for i in range(len(parts))]
print(f"\n\n")
print(f"<mujoco>")
print(f"  <asset>")
print(f'    <mesh name="{obj_name}_visual_mesh" file="../../meshes/{obj_name}.STL"/>')

# 添加所有分解后的碰撞模型
for i, filename in enumerate(part_filenames):
    # 给每个网格起个唯一的 name ID
    print(f'    <mesh name="{obj_name}_col_{i}" file="../../meshes/parts/{filename}"/>')
print(f"  </asset>\n")

# --- 生成 Body 部分 ---
print(f"  <worldbody>")
print(f'    <body name="{obj_name}" pos="0 0 0">')
# (1) 视觉 Geom (通常 Group 1用于视觉，不参与计算)
print(f'      ')
print(f'      <geom type="mesh" mesh="{obj_name}_visual_mesh" group="1" material="some_mat"/>')

# (2) 碰撞 Geoms (Group 3 默认用于碰撞，或者默认 Group 0)
# 关键点：不需要 pos/quat，默认就是相对于 body 原点 (0,0,0)
print(f'      ')
for i in range(len(parts)):
    # 这里的 class="collision" 是可选的，用于统一管理颜色或物理属性
    print(f'      <geom type="mesh" mesh="{obj_name}_col_{i}" group="3"/>') # group=3 通常是仅碰撞不可见
    # 如果你想调试看清楚这些凸包，可以把 group="3" 去掉，加 rgba="1 0 0 0.5"
    
print(f"    </body>")
print(f"  </worldbody>")