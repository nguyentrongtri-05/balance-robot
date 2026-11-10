function xecanbang3d(arg1, arg2)
    persistent log_x log_theta timer_auto_open

    %% 1. CHẾ ĐỘ GHI DỮ LIỆU TỪ SIMULINK (CHẠY SIÊU TỐC)
    if nargin == 2
        log_x(end+1, 1)     = arg1;
        log_theta(end+1, 1) = arg2;

        if isempty(timer_auto_open) || ~isvalid(timer_auto_open)
            timer_auto_open = timer('StartDelay', 0.25, ...
                'TimerFcn', @(~,~) xecanbang3d());
            start(timer_auto_open);
        else
            stop(timer_auto_open);
            start(timer_auto_open);
        end
        return;
    end

    %% 2. KHI SIMULINK CHẠY XONG -> XỬ LÝ DỮ LIỆU ĐỂ MỞ TRÌNH TUA VIDEO
    if ~isempty(timer_auto_open) && isvalid(timer_auto_open)
        stop(timer_auto_open);
        delete(timer_auto_open);
    end

    % Lấy thời gian mô phỏng thực từ Simulink (mặc định 10s nếu không đọc được)
    sim_stop_time = 10;
    try
        if ~isempty(bdroot)
            sim_stop_time = str2double(get_param(bdroot, 'StopTime'));
        end
    catch
    end
    if isnan(sim_stop_time) || sim_stop_time <= 0, sim_stop_time = 10; end

    if nargin == 1 && (isstruct(arg1) || isa(arg1, 'timeseries'))
        t_raw = arg1.Time;
        y_raw = squeeze(arg1.Data);
        if size(y_raw, 1) == 2 && size(y_raw, 2) ~= 2, y_raw = y_raw'; end
        x_raw = y_raw(:,1);
        theta_raw = y_raw(:,2);
    elseif ~isempty(log_x)
        num_pts = length(log_x);
        t_raw = linspace(0, sim_stop_time, num_pts)';
        x_raw = log_x;
        theta_raw = log_theta;
        log_x = [];
        log_theta = [];
    else
        t_raw = (0:0.02:5)';
        x_raw = zeros(size(t_raw));
        theta_raw = zeros(size(t_raw));
    end

    % Nội suy 250 khung hình để kéo thanh tua mượt mà
    if length(t_raw) < 2
        t_play = [0; 1];
        x_play = [x_raw(1); x_raw(1)];
        theta_play = [theta_raw(1); theta_raw(1)];
    else
        num_frames = 250;
        t_play = linspace(t_raw(1), t_raw(end), num_frames)';
        t_in   = linspace(t_raw(1), t_raw(end), length(x_raw))';
        x_play     = interp1(t_in, x_raw, t_play, 'linear');
        theta_play = interp1(t_in, theta_raw, t_play, 'linear');
    end
    num_frames = length(t_play);

    %% 3. TẠO GIAO DIỆN TRÌNH PHÁT VIDEO 3D + ĐỒ THỊ
    old_fig = findobj('Type', 'figure', 'Name', 'Trinh Phat & Tua Video 3D - Xe Tu Can Bang');
    if ~isempty(old_fig), close(old_fig); end

    fig = figure('Color', [0.95 0.96 0.98], ...
                 'Name', 'Trinh Phat & Tua Video 3D - Xe Tu Can Bang', ...
                 'Position', [80, 60, 1220, 740]);

    % Khung bên trái: Mô hình 3D
    ax = axes('Parent', fig, 'Position', [0.05, 0.22, 0.52, 0.72], 'Color', 'w');
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on'); axis(ax, 'equal');
    view(ax, 135, 22);
    xlabel(ax, 'Trục X (mm)', 'FontWeight', 'bold');
    ylabel(ax, 'Hướng chạy Y (mm)', 'FontWeight', 'bold');
    zlabel(ax, 'Độ cao Z (mm)', 'FontWeight', 'bold');
    light(ax, 'Position', [1000 1000 1500], 'Style', 'infinite');
    light(ax, 'Position', [-1000 -1000 800], 'Style', 'infinite');
    lighting(ax, 'gouraud');
    rotate3d(ax, 'on');

    % Khung bên phải: Đồ thị đáp ứng
    ax_plot = axes('Parent', fig, 'Position', [0.63, 0.22, 0.34, 0.72]);
    hold(ax_plot, 'on'); grid(ax_plot, 'on'); box(ax_plot, 'on');
    plot(ax_plot, t_play, rad2deg(theta_play), 'r', 'LineWidth', 1.8);
    plot(ax_plot, t_play, x_play * 100, 'b', 'LineWidth', 1.8);
    y_lim_plot = ylim(ax_plot);
    if y_lim_plot(1) == y_lim_plot(2), y_lim_plot = [-10 10]; ylim(ax_plot, y_lim_plot); end
    vach_thoi_gian = line(ax_plot, [0 0], y_lim_plot, 'Color', 'k', 'LineWidth', 2, 'LineStyle', '--');
    title(ax_plot, 'Đồ thị Đáp ứng (Kéo thanh tua bên dưới để xem)');
    xlabel(ax_plot, 'Thời gian (s)'); ylabel(ax_plot, 'Giá trị');
    legend(ax_plot, {'Góc nghiêng \theta (độ)', 'Vị trí x (cm)', 'Thời điểm hiện tại'}, 'Location', 'best');

    %% 4. ĐỌC FILE STL GỐC & ĐẶT XE CHẠM SÀN Z = 0
    fv_than     = stlread('thanxe.STL');
    fv_banhphai = stlread('banhphai.STL');
    fv_banhtrai = stlread('banhtrai.STL');

    all_pts = [fv_than.Points; fv_banhphai.Points; fv_banhtrai.Points];
    z_min = min([fv_banhphai.Points(:,3); fv_banhtrai.Points(:,3)]);
    z_max = max(all_pts(:,3)) - z_min;
    z_offset = -z_min; % Đưa đáy bánh xe chạm chuẩn mặt sàn Z = 0

    kich_thuoc_stl = max(max(all_pts) - min(all_pts));
    scale_unit = (kich_thuoc_stl > 10) * 999 + 1; % 1000 nếu STL là mm

    tam_phai = mean(fv_banhphai.Points, 1);
    tam_trai = mean(fv_banhtrai.Points, 1);
    tam_truc = (tam_phai + tam_trai) / 2;
    R_banh   = max(0.03, (tam_truc(3) - z_min) / scale_unit);

    vec_truc = abs(tam_trai(1:2) - tam_phai(1:2));
    [~, idx_truc] = max(vec_truc);
    cac_truc_xoay = {'xrotate', 'yrotate'};
    truc_xoay_banh = cac_truc_xoay{idx_truc};
    truc_tinh_tien = 3 - idx_truc;

    % Vẽ mặt sàn tự động trải dài theo toàn bộ quãng đường xe đi
    min_dist = min(0, min(x_play) * scale_unit) - 500;
    max_dist = max(0, max(x_play) * scale_unit) + 500;
    san_rong = 400; o_gach = 100;
    
    start_gy = floor(min_dist / o_gach) * o_gach;
    end_gy   = ceil(max_dist / o_gach) * o_gach;

    for gx = -san_rong : o_gach : san_rong - o_gach
        for gy = start_gy : o_gach : end_gy - o_gach
            mau_o = [0.91 0.93 0.95] * (mod(round((gx+gy)/o_gach), 2)==0) + ...
                    [0.82 0.85 0.88] * (mod(round((gx+gy)/o_gach), 2)~=0);
            if truc_tinh_tien == 2
                patch(ax, 'XData', [gx, gx+o_gach, gx+o_gach, gx] + tam_truc(1), ...
                          'YData', [gy, gy, gy+o_gach, gy+o_gach] + tam_truc(2), ...
                          'ZData', [0, 0, 0, 0], 'FaceColor', mau_o, 'EdgeColor', 'none');
            else
                patch(ax, 'XData', [gy, gy+o_gach, gy+o_gach, gy] + tam_truc(1), ...
                          'YData', [gx, gx, gx+o_gach, gx+o_gach] + tam_truc(2), ...
                          'ZData', [0, 0, 0, 0], 'FaceColor', mau_o, 'EdgeColor', 'none');
            end
        end
    end

    % Vạch xuất phát tại x = 0
    line(ax, [-san_rong, san_rong] + tam_truc(1), [tam_truc(2), tam_truc(2)], [0.5, 0.5], ...
         'Color', 'r', 'LineWidth', 2.5, 'LineStyle', '--');

    % Vẽ 3 khối STL của xe
    Group_Xe       = hgtransform('Parent', ax);
    Group_Than     = hgtransform('Parent', Group_Xe);
    Group_BanhPhai = hgtransform('Parent', Group_Xe);
    Group_BanhTrai = hgtransform('Parent', Group_Xe);

    patch(ax, 'Faces', fv_than.ConnectivityList, 'Vertices', fv_than.Points, ...
          'Parent', Group_Than, 'FaceColor', [0.12 0.26 0.82], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
    patch(ax, 'Faces', fv_banhphai.ConnectivityList, 'Vertices', fv_banhphai.Points, ...
          'Parent', Group_BanhPhai, 'FaceColor', [0.18 0.18 0.20], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
    patch(ax, 'Faces', fv_banhtrai.ConnectivityList, 'Vertices', fv_banhtrai.Points, ...
          'Parent', Group_BanhTrai, 'FaceColor', [0.18 0.18 0.20], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

    zlim(ax, [0, z_max * 1.35]); % Bỏ hoàn toàn trục Z âm

    %% 5. THANH TRƯỢT TUA VIDEO & NÚT PLAY/PAUSE
    dang_phat = false;
    frame_hien_tai = 1;

    btn_play = uicontrol(fig, 'Style', 'pushbutton', 'Position', [60, 45, 120, 38], ...
                         'String', '▶ PHÁT (Play)', 'FontSize', 11, 'FontWeight', 'bold', ...
                         'BackgroundColor', [0.15 0.65 0.30], 'ForegroundColor', 'w', ...
                         'Callback', @bam_nut_play);

    uicontrol(fig, 'Style', 'text', 'Position', [195, 50, 65, 22], ...
              'String', 'Tốc độ:', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.95 0.96 0.98]);
    menu_toc_do = uicontrol(fig, 'Style', 'popupmenu', 'Position', [260, 52, 90, 25], ...
                            'String', {'0.25x (Chậm)', '0.5x', '1.0x (Chuẩn)', '2.0x (Nhanh)', '4.0x'}, ...
                            'Value', 3, 'FontSize', 10);

    sld_tua = uicontrol(fig, 'Style', 'slider', 'Position', [365, 52, 610, 24], ...
                        'Min', 1, 'Max', num_frames, 'Value', 1, ...
                        'SliderStep', [1/max(1,num_frames-1), 10/max(1,num_frames-1)]);
    addlistener(sld_tua, 'ContinuousValueChange', @(~, ~) cap_nhat_khung_hinh(round(get(sld_tua, 'Value'))));

    lbl_time = uicontrol(fig, 'Style', 'text', 'Position', [990, 48, 180, 26], ...
                         'String', sprintf('0.00 s / %.2f s', t_play(end)), ...
                         'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', [0.95 0.96 0.98]);

    % Hiển thị ngay ở Frame 1 (lúc t = 0)
    cap_nhat_khung_hinh(1);

    function cap_nhat_khung_hinh(idx)
        idx = max(1, min(num_frames, round(idx)));
        frame_hien_tai = idx;

        t_now     = t_play(idx);
        x_m       = x_play(idx);
        theta_rad = theta_play(idx);
        phi_banh  = x_m / R_banh;

        T_than = makehgtform('translate', tam_truc) * ...
                 makehgtform(truc_xoay_banh, theta_rad) * ...
                 makehgtform('translate', -tam_truc);
        set(Group_Than, 'Matrix', T_than);

        T_phai = makehgtform('translate', tam_phai) * ...
                 makehgtform(truc_xoay_banh, phi_banh) * ...
                 makehgtform('translate', -tam_phai);
        set(Group_BanhPhai, 'Matrix', T_phai);

        T_trai = makehgtform('translate', tam_trai) * ...
                 makehgtform(truc_xoay_banh, phi_banh) * ...
                 makehgtform('translate', -tam_trai);
        set(Group_BanhTrai, 'Matrix', T_trai);

        vec_tinh_tien = [0, 0, z_offset];
        vec_tinh_tien(truc_tinh_tien) = x_m * scale_unit;
        set(Group_Xe, 'Matrix', makehgtform('translate', vec_tinh_tien));

        % KHÓA CAMERA BÁM THEO XE: Luôn giữ xe nằm ngay giữa khung nhìn 3D!
        vi_tri_xe_mm = tam_truc(truc_tinh_tien) + x_m * scale_unit;
        if truc_tinh_tien == 2
            xlim(ax, [tam_truc(1) - 350, tam_truc(1) + 350]);
            ylim(ax, [vi_tri_xe_mm - 450, vi_tri_xe_mm + 450]);
        else
            xlim(ax, [vi_tri_xe_mm - 450, vi_tri_xe_mm + 450]);
            ylim(ax, [tam_truc(2) - 350, tam_truc(2) + 350]);
        end

        set(vach_thoi_gian, 'XData', [t_now, t_now]);
        set(lbl_time, 'String', sprintf('%.2f s / %.2f s', t_now, t_play(end)));
        title(ax, sprintf('Vị trí x = %.3f m  |  Góc nghiêng \\theta = %.2f°', x_m, rad2deg(theta_rad)), ...
              'FontSize', 12, 'FontWeight', 'bold');
        drawnow;
    end

    function bam_nut_play(~, ~)
        dang_phat = ~dang_phat;
        if dang_phat
            set(btn_play, 'String', '⏸ DỪNG (Pause)', 'BackgroundColor', [0.85 0.35 0.15]);
            if frame_hien_tai >= num_frames, frame_hien_tai = 1; end
            cac_he_so = [0.25, 0.5, 1.0, 2.0, 4.0];
            while dang_phat && ishandle(fig) && frame_hien_tai <= num_frames
                toc_do = cac_he_so(get(menu_toc_do, 'Value'));
                buoc_nhay = max(1, round(2 * toc_do));
                set(sld_tua, 'Value', frame_hien_tai);
                cap_nhat_khung_hinh(frame_hien_tai);
                frame_hien_tai = frame_hien_tai + buoc_nhay;
                pause(0.015 / max(1, toc_do));
            end
            if ishandle(fig)
                dang_phat = false;
                set(btn_play, 'String', '▶ PHÁT (Play)', 'BackgroundColor', [0.15 0.65 0.30]);
            end
        else
            set(btn_play, 'String', '▶ PHÁT (Play)', 'BackgroundColor', [0.15 0.65 0.30]);
        end
    end
end