function posEst = linearTriangulation(locatorPos, anglesGlobal, weights)
    % wlsPositionEstimate3D: 방위각(Azimuth) 및 고도각(Elevation)을 이용한 
    %                        가중 최소제곱법(Weighted Least Squares) 기반 3D 선형 위치 추정
    %
    % 입력:
    %   locatorPos   - [3 x N] 앵커 좌표 (x, y, z)
    %   anglesGlobal - [N x 2] 추정된 글로벌 방향각 (도 단위) [Azimuth, Elevation]
    %   weights      - (선택 사항) [N x 1] 각 앵커에 대한 신뢰도 가중치 배열. 
    %                  입력을 생략하거나 빈 배열([])을 넣으면 가중치 없는 일반 삼각측량 수행.
    
    N = size(locatorPos, 2);
    
    % ---------------------------------------------------------------------
    % [추가] 가중치 인자가 없거나 비어있는 경우, 모든 가중치를 1로 설정 (일반 삼각측량)
    % ---------------------------------------------------------------------
    if nargin < 3 || isempty(weights)
        weights = ones(N, 1);
    end
    
    % 선형 방정식을 구성할 행렬(A) 및 벡터(b) 초기화
    % 각 앵커당 x, y, z 방향에 대한 2개의 제약 방정식이 나오므로 크기는 [2N x 3]
    A = zeros(2 * N, 3);
    b = zeros(2 * N, 1);
    W = zeros(2 * N, 2 * N); % 가중치 대각 행렬
    
    for i = 1:N
        % 각도 라디안 변환
        az = deg2rad(anglesGlobal(i, 1));
        el = deg2rad(anglesGlobal(i, 2));
        
        % 타겟을 향하는 방향 벡터 계산 (Direction Cosines)
        dx = cos(el) * cos(az);
        dy = cos(el) * sin(az);
        dz = sin(el);
        
        % 앵커 위치
        x_i = locatorPos(1, i);
        y_i = locatorPos(2, i);
        z_i = locatorPos(3, i);
        
        % 3D 선형 방정식 유도:
        % (y - y_i)*dx = (x - x_i)*dy  --> -dy*x + dx*y + 0*z = dx*y_i - dy*x_i
        % (z - z_i)*dx = (x - x_i)*dz  --> -dz*x + 0*y + dx*z = dx*z_i - dz*x_i
        
        row1 = 2*i - 1;
        row2 = 2*i;
        
        A(row1, :) = [-dy, dx, 0];
        b(row1)    = dx * y_i - dy * x_i;
        
        A(row2, :) = [-dz, 0, dx];
        b(row2)    = dx * z_i - dz * x_i;
        
        % 가중치 행렬 구성 (각 앵커의 2개 방정식에 동일한 가중치 적용)
        W(row1, row1) = weights(i);
        W(row2, row2) = weights(i);
    end
    
    % 가중 최소제곱법(WLS) 해 계산: x = (A' * W * A) \ (A' * W * b)
    % (weights가 모두 1이면 일반 OLS와 완전히 동일한 결과가 도출됨)
    posEst = (A' * W * A) \ (A' * W * b);
end