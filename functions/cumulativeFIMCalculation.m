function FIM = cumulativeFIMCalculation(nodePos, locatorPos, locatorRotMat, antennaPos, lambda, SNR_Lin)
    % cumulativeFIMCalculation: 여러 로케이터(앵커)의 정보를 취합하여 전체 시스템의 
    %                           Fisher Information Matrix(FIM) 및 최적성 지표를 계산합니다.
    % 
    % 입력 파라미터:
    %   nodePos     : [1x3] 또는 [3x1] 타겟 노드의 3D 글로벌 좌표 (x, y, z)
    %   locatorPos  : [Nx3] N개 로케이터의 3D 글로벌 좌표
    %   locatorQuat : [Nx4] N개 로케이터의 자세를 나타내는 쿼터니언 (Local -> Global 회전)
    %   antennaPos  : [3xM] 안테나 배열 내 M개 소자의 로컬 3D 좌표
    %   lambda      : [스칼라] RF 신호의 파장
    %   SNR_Lin     : [Nx1] 또는 [1xN] 각 로케이터마다 경로 감쇠가 이미 적용된 선형 스케일의 SNR 배열
    
    % 로케이터(앵커)의 총 개수 N 확인
    N = size(locatorPos, 1);
    
    % 전체 시스템의 FIM 누적을 위한 3x3 영행렬 초기화
    I_total = zeros(3, 3);
    
    % 계산 편의를 위해 타겟 위치를 열 벡터로 변환
    x_target = nodePos(:);

    % 안테나 길이에 파장 반영
    antennaPos = antennaPos * lambda;
    
    for i = 1:N
        % i번째 로케이터의 위치(열 벡터)와 쿼터니언 추출
        loc_i = locatorPos(i, :)';

        % 1. 글로벌 좌표계의 벡터를 로케이터의 로컬 좌표계로 변환
        v_global = x_target - loc_i; % 로케이터에서 타겟을 향하는 글로벌 벡터
        
        % quat2rotm은 기본적으로 Local -> Global 변환 행렬을 반환함
        M_local_to_global = locatorRotMat(:, :, i);
        
        % Global -> Local 변환을 위해 직교행렬의 특성을 이용해 전치(역행렬)를 취함
        M_global_to_local = M_local_to_global.'; 
        
        % 타겟 벡터를 로케이터 기준의 로컬 좌표(x_l, y_l, z_l)로 회전 변환
        v_local = M_global_to_local * v_global; 
        
        xl = v_local(1);
        yl = v_local(2);
        zl = v_local(3);
        
        % 2. 앵커별로 미리 계산된 거리 감쇠 SNR 할당
        % (외부에서 Path Loss 모델 등을 거쳐 계산된 배열을 그대로 사용)
        SNR_lin_i = SNR_Lin(i);
        
        % 3. 로컬 좌표계를 기반으로 구면 좌표계의 방위각(phi) 및 고도각(psi) 도출
        % phi(방위각): X-Y 평면에서의 각도
        phi_i = atan2(yl, xl);
        % psi(고도각): X-Y 평면과 Z축 사이의 각도
        psi_i = atan(zl / sqrt(xl^2 + yl^2));
        
        % 4. 최적화된 하위 함수 호출
        % 타겟 위치의 미세한 변화가 측정 각도에 미치는 영향을 나타내는 Jacobian 행렬 계산
        J_i = calculate_Jacobian(v_local, M_local_to_global);
        
        % 해당 입사각(phi, psi)과 SNR에서 안테나 배열이 갖는 각도 추정 공분산의 역행렬(각도 FIM) 계산
        I_AOA_i = calculate_Covariance(phi_i, psi_i, antennaPos, lambda, SNR_lin_i);
        
        % 5. 연쇄 법칙(Chain Rule)을 적용하여 개별 로케이터의 3D 위치 추정 FIM 계산
        % I_i = J_i^T * R_i^-1 * J_i (여기서 R_i^-1 은 I_AOA_i 와 동일)
        I_i = J_i.' * I_AOA_i * J_i;
        
        % 전체 시스템의 FIM에 선형 합산 (정보는 더해질수록 확실해짐)
        I_total = I_total + I_i;
    end

    FIM = I_total;
end

function J = calculate_Jacobian(v_local, M_local_to_global)
    % calculate_Jacobian: 타겟의 3D 글로벌 위치 변화가 로케이터의 각도 측정값(방위각, 고도각)에 
    %                     미치는 변화율(편미분)을 계산합니다.
    
    % 인자로 넘겨받은 로컬 좌표 성분 분리
    xl = v_local(1);
    yl = v_local(2);
    zl = v_local(3);
    
    % 삼각함수 미분에 공통으로 반복 사용되는 거리항 미리 계산
    r2_xy = xl^2 + yl^2;          % X-Y 평면 투영 거리의 제곱
    r_xy = sqrt(r2_xy);           % X-Y 평면 투영 거리
    r2_xyz = r2_xy + zl^2;        % 3D 원점으로부터의 직선 거리 제곱
    
    % [1] 방위각(phi)에 대한 편미분 (x_local, y_local, z_local 방향)
    dphi_dx = -yl / r2_xy;
    dphi_dy = xl / r2_xy;
    dphi_dz = 0;                  % 방위각은 Z축 변화에 영향을 받지 않음
    
    % [2] 고도각(psi)에 대한 편미분 (x_local, y_local, z_local 방향)
    dpsi_dx = -(xl * zl) / (r2_xyz * r_xy);
    dpsi_dy = -(yl * zl) / (r2_xyz * r_xy);
    dpsi_dz = r_xy / r2_xyz;
    
    % 로컬 좌표에 대한 각도(phi, psi)의 Jacobian 조립 (2x3 크기)
    J_local = [dphi_dx, dphi_dy, dphi_dz;
               dpsi_dx, dpsi_dy, dpsi_dz];
           
    % [3] 좌표계 회전 변환
    % 구한 미분값은 '로컬 좌표' 기준이므로, 이를 다시 '글로벌 좌표' 기준으로 변환해야 함
    M_global_to_local = M_local_to_global.'; % Global -> Local 회전 행렬
    
    % 연쇄 법칙: (각도의 로컬 좌표 미분) * (글로벌 좌표를 로컬로 바꾸는 회전 변환)
    J = J_local * M_global_to_local;
end

function I_AOA = calculate_Covariance(phi, psi, antennaPos, lambda, SNR_lin)
    % calculate_Covariance: 주어진 입사각과 안테나 기하 구조, SNR을 바탕으로 
    %                       각도 추정의 Fisher Information Matrix (I_AOA)를 계산합니다.
    %                       (Cramer-Rao Lower Bound 역수에 해당)
    
    % 안테나 위치 정보를 각 축 성분별로 분리
    xm = antennaPos(1, :);
    ym = antennaPos(2, :);
    zm = antennaPos(3, :);
    
    % 파동수(Wavenumber) 계산 (단위 거리당 위상 변화량)
    k = 2 * pi / lambda;
    
    % 입사각 방향의 단위 벡터가 각 안테나 소자에 도달할 때 발생하는 
    % 물리적인 경로차(Path Difference)를 방위각(phi)과 고도각(psi)에 대해 편미분
    dp_dphi = xm .* (-cos(psi) * sin(phi)) + ym .* (cos(psi) * cos(phi));
    dp_dpsi = xm .* (-sin(psi) * cos(phi)) + ym .* (-sin(psi) * sin(phi)) + zm .* cos(psi);
    
    % Steering Vector(조향 벡터) 미분의 복소수 연산을 실수 편미분의 제곱 형태로 단순화
    % (각도 정보량은 SNR, 파동수 제곱, 그리고 센서 배열의 물리적 분산도에 비례함)
    I11 = SNR_lin * (k^2) * sum(dp_dphi .^ 2);        % 방위각(phi) 정보량
    I22 = SNR_lin * (k^2) * sum(dp_dpsi .^ 2);        % 고도각(psi) 정보량
    I12 = SNR_lin * (k^2) * sum(dp_dphi .* dp_dpsi);  % 두 각도 간의 교차 상관 관계
    
    % 2x2 크기의 각도 추정 FIM 조립 (대칭 행렬)
    I_AOA = [I11, I12; 
             I12, I22];
end