model_name = 'Xe_Cascade_PID';
open_system(model_name);

% 1. Xóa khối Hien_Thi_3D gây chậm khi đang chạy Simulink
if ~isempty(find_system(model_name, 'Name', 'Hien_Thi_3D'))
    delete_line(model_name, 'Demux/1', 'Hien_Thi_3D/1');
    delete_line(model_name, 'Demux/2', 'Hien_Thi_3D/2');
    delete_block([model_name '/Hien_Thi_3D']);
end

% 2. Thêm khối To Workspace để lưu dữ liệu [x, theta] tốc độ cao
if isempty(find_system(model_name, 'Name', 'Luu_Du_Lieu'))
    add_block('simulink/Sinks/To Workspace', [model_name '/Luu_Du_Lieu'], ...
              'VariableName', 'sim_data', ...
              'SaveFormat', 'Timeseries', ...
              'Position', [620, 145, 700, 175]);
    add_line(model_name, 'Dong_Luc_Hoc_Xe/1', 'Luu_Du_Lieu/1');
end

% 3. Cài đặt tự động gọi trình phát 3D có thanh tua ngay khi Simulink chạy xong
set_param(model_name, 'ReturnWorkspaceOutputs', 'off');
set_param(model_name, 'StopFcn', 'xecanbang3d(sim_data);');
save_system(model_name);
disp('Da cap nhat xong! Bay gio ban bam Run trong Simulink se chay tuc thi!');