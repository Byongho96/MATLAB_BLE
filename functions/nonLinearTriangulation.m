function posNonLin = nonLinearTriangulation(posLin, validLocs, validAngles, weights)
    % 입력인자:
    %   posLin      - 선형 삼각측량으로 구한 초기 추정 위치 [x; y; z]
    %   validLocs   - 측위에 사용된 유효 로케이터들의 위치 행렬 [3 x N]
    %   validAngles - 측위된 글로벌 방위각/고도각 행렬 [N x 2]
    %   weights     - (선택 사항) [N x 1] 각 앵커에 대한 신뢰도 가중치 배열. 
    %                 입력을 생략하거나 빈 배열([])을 넣으면 가중치 없는 일반 최적화 수행.
    
    % ---------------------------------------------------------------------
    % [추가] 가중치 인자가 없거나 비어있는 경우, 모든 가중치를 1로 설정
    % ---------------------------------------------------------------------
    N = size(validLocs, 2);
    if nargin < 4 || isempty(weights)
        weights = ones(N, 1);
    end
    
    % 1. 최적화 옵션 설정 (콘솔 출력 끄기, LM 알고리즘 지정)
    options = optimoptions('lsqnonlin', 'Display', 'off', 'Algorithm', 'levenberg-marquardt');
    
    % 2. 잔차(Residual) 계산을 위한 익명 함수 정의
    % 내부 함수인 computeAngleResiduals에 weights 배열도 함께 전달
    residualFunc = @(p) computeAngleResiduals(p, validLocs, validAngles, weights);
    
    % 3. 비선형 최소자승법 실행 (posLin을 훌륭한 '초기값'으로 사용)
    try
        [posEst, ~, ~, exitflag] = lsqnonlin(residualFunc, posLin, [], [], options);
        
        % exitflag > 0 이면 최적화가 성공적으로 수렴했다는 뜻
        if exitflag > 0
            posNonLin = posEst;
        else
            % 수렴 실패(반복 횟수 초과 등) 시 튀는 것을 방지하기 위해 선형 결과 유지
            posNonLin = posLin; 
        end
    catch
        % 행렬 연산 에러 등 예기치 못한 오류 발생 시 안전하게 선형 결과 유지
        posNonLin = posLin; 
    end
    
    % ---------------------------------------------------------------------
    % 내부 중첩 함수 (Nested Function): 각도 잔차 계산
    % ---------------------------------------------------------------------
    function res = computeAngleResiduals(p, locs, anglesGt, w)
        numLocs = size(locs, 2);
        res = zeros(numLocs * 2, 1); 
        
        for i = 1:numLocs
            % 로케이터에서 현재 탐색 위치(p)를 향하는 3D 벡터
            dirVec = p - locs(:, i);
            
            % 기하학적 각도 계산
            [az_rad, el_rad, ~] = cart2sph(dirVec(1), dirVec(2), dirVec(3));
            az_exp = rad2deg(az_rad);
            el_exp = rad2deg(el_rad);
            
            % 측정된 각도
            az_meas = anglesGt(i, 1);
            el_meas = anglesGt(i, 2);
            
            % 각도 래핑 (360도 경계선 오차 방지: 무조건 -180 ~ 180도로 맞춤)
            az_diff = mod((az_meas - az_exp) + 180, 360) - 180;
            el_diff = mod((el_meas - el_exp) + 180, 360) - 180;
            
            % -------------------------------------------------------------
            % [수정] 잔차에 가중치의 제곱근(sqrt)을 곱하여 적재
            % lsqnonlin이 내부적으로 이 값을 제곱(sum(res.^2))하므로, 
            % 결과적으로 가중치(w)가 곱해진 오차 제곱합을 최소화하게 됨
            % -------------------------------------------------------------
            res(2*i - 1) = az_diff * sqrt(w(i));
            res(2*i)     = el_diff * sqrt(w(i));
        end
    end
end