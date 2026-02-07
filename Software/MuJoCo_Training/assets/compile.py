import mujoco
import os

def compile_with_vfs():
    # 1. Định vị các đường dẫn
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "robot_cleaned.urdf")
    mesh_dir = os.path.join(current_dir, "meshes")
    output_xml = os.path.join(current_dir, "robot_humanoid.xml")

    # 2. Tạo Virtual File System (VFS)
    vfs = mujoco.MjVFS()

    # 3. Tự động tìm tất cả file .STL trong thư mục meshes và nạp vào VFS
    print("📂 Đang nạp các file mesh vào bộ nhớ ảo...")
    mesh_files = [f for f in os.listdir(mesh_dir) if f.endswith('.STL')]
    
    for mesh_name in mesh_files:
        full_path = os.path.join(mesh_dir, mesh_name)
        # Nạp vào VFS với tên rút gọn (để URDF dễ gọi)
        vfs.add_file(full_path, mesh_name)
        print(f"   ✅ Đã nạp: {mesh_name}")

    # 4. Đọc file URDF và sửa nội dung để gọi Mesh từ VFS (không dùng đường dẫn)
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # Sửa filename="meshes/XXX.STL" thành filename="XXX.STL" 
    # Vì trong VFS chúng ta đã đặt tên trực tiếp là XXX.STL
    urdf_content = urdf_content.replace('filename="meshes/', 'filename="')

    print("🚀 Đang biên dịch từ bộ nhớ ảo...")
    try:
        # Biên dịch từ string kèm theo hệ thống file ảo VFS
        model = mujoco.MjModel.from_xml_string(urdf_content, vfs=vfs)
        
        # Lưu lại file XML (MuJoCo sẽ tự nhúng đường dẫn tương đối sạch vào đây)
        mujoco.mj_saveLastXML(output_xml, model)
        
        print(f"🎉 THÀNH CÔNG! Đã tạo file: {output_xml}")
        print("👉 Bây giờ hãy thử mở viewer: python3 -m mujoco.viewer --mjcf robot_humanoid.xml")
        
    except Exception as e:
        print("\n❌ Lỗi biên dịch:")
        print(e)

if __name__ == "__main__":
    compile_with_vfs()