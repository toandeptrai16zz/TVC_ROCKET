% ==========================================
% THÔNG SỐ VẬT LÝ TÊN LỬA
% Trích xuất và xấp xỉ từ OpenRocket
% ==========================================
m = 0.605;          % Khối lượng tổng của tên lửa, đơn vị kg
L = 0.55;           % Chiều dài tổng của tên lửa, đơn vị m
r = 0.025;          % Bán kính thân tên lửa, đơn vị m
F_thrust = 15;      % Lực đẩy motor ước tính, đơn vị Newton
g = 9.81;           % Gia tốc trọng trường, đơn vị m/s^2

% ==========================================
% MÔ-MEN QUÁN TÍNH
% ==========================================
Ixx = 0.5 * m * r^2;
Iyy = (1/12) * m * (3*r^2 + L^2);
Izz = Iyy;

% ==========================================
% GIỚI HẠN CƠ CẤU CHẤP HÀNH
% ==========================================
servo_max =  15;    % Góc bẻ TVC tối đa (độ)
servo_min = -15;    % Góc bẻ TVC tối thiểu (độ)

% ==========================================
% TÍNH PID TỪ ĐẶC TÍNH VẬT LÝ
% ==========================================

% Cánh tay đòn TVC (khoảng cách từ nozzle đến tâm khối lượng)
L_tvc = L * 0.45;   % ≈ 45% chiều dài tên lửa, đơn vị m

% Hệ số khuếch đại plant (pitch dynamics)
K_plant = (F_thrust * L_tvc) / Iyy;

% Thông số mong muốn của hệ thống
wn   = 5.0;         % Tần số tự nhiên (rad/s) — phản hồi nhanh
zeta = 0.7;         % Hệ số tắt dần — ít overshoot, ổn định tốt

% Tính Kp, Kd theo pole placement
% Từ phương trình đặc trưng: s^2 + 2*zeta*wn*s + wn^2 = 0
Kp = (wn^2) / K_plant;
Kd = (2 * zeta * wn) / K_plant;
Ki = 0.01;          % Nhỏ để tránh tích phân gây vọt lố

% ==========================================
% THÔNG SỐ BỘ ĐIỀU KHIỂN ĐỘ CAO
% ==========================================
wn_alt   = 2.0;     % Tần số tự nhiên vòng độ cao (chậm hơn vòng góc)
zeta_alt = 0.8;     % Hệ số tắt dần — ưu tiên ổn định hơn tốc độ

% Hệ số khuếch đại vòng độ cao
K_alt = (F_thrust - m*g) / m;   % gia tốc dư sau khi bù trọng lực

Kp_alt = (wn_alt^2) / K_alt;
Kd_alt = (2 * zeta_alt * wn_alt) / K_alt;
Ki_alt = 0.005;

% Tự động nạp vào Simulink
set_param('TVC_Rocket/PID Controller','P',num2str(Kp),'I',num2str(Ki),'D',num2str(Kd));
set_param('TVC_Rocket/PID Altitude','P',num2str(Kp_alt),'I',num2str(Ki_alt),'D',num2str(Kd_alt));
% ==========================================
% IN KẾT QUẢ
% ==========================================
fprintf('\n===== THÔNG SỐ HỆ THỐNG =====\n');
fprintf('Iyy       = %.6f kg.m2\n', Iyy);
fprintf('L_tvc     = %.4f m\n', L_tvc);
fprintf('K_plant   = %.4f\n', K_plant);
fprintf('\n===== PID VÒNG GÓC (PITCH) =====\n');
fprintf('Kp = %.6f\n', Kp);
fprintf('Ki = %.6f\n', Ki);
fprintf('Kd = %.6f\n', Kd);
fprintf('\n===== PID VÒNG ĐỘ CAO =====\n');
fprintf('Kp_alt = %.6f\n', Kp_alt);
fprintf('Ki_alt = %.6f\n', Ki_alt);
fprintf('Kd_alt = %.6f\n', Kd_alt);
fprintf('==============================\n');