function posEst = nonLinearTriangulation(locators, measuredAngles, initialPos)
    % NONLINEARTRIANGULATION 비선형 최소제곱법을 이용한 3D 위치 추정 (가중치 미적용)
    %
    % [입력 파라미터]
    % - locators: 활성 로케이터 위치 (3 x N 행렬)
    % - measuredAngles: 측정된 [Azimuth, Elevation] (N x 2 행렬, 단위: 도)
    % - initialPos: 선형 방식으로 계산된 초기 위치 [x; y; z] (3 x 1 벡터)
    % - roomSize: 시뮬레이션 공간 크기 [L, W, H] (경계 조건 설정용)

    % 1. 최적화 옵션 설정
    options = optimoptions('lsqnonlin', ...
        'Display', 'off', ...
        'Algorithm', 'trust-region-reflective', ...
        'FunctionTolerance', 1e-6, ...
        'StepTolerance', 1e-6);

    % 3. 비선형 최적화 수행
    % 가중치 없이 각도 잔차만 전달합니다.
    try
        posEst = lsqnonlin(@(p) angleResiduals(p, locators, measuredAngles), ...
                           initialPos, [], [], options);
    catch
        % 최적화 수렴 실패 시 초기값(선형 추정값) 유지
        posEst = initialPos;
    end
end

function res = angleResiduals(p, locators, measuredAngles)
    % 잔차 계산을 위한 내부 함수
    numLocs = size(locators, 2);
    res = zeros(numLocs * 2, 1);
    
    for i = 1:numLocs
        % 현재 추정 위치 p에서 로케이터 i를 바라보는 상대 벡터
        relVec = p - locators(:, i);
        
        % 이론적 각도 계산 (Cartesian -> Spherical)
        [calcAzRad, calcElRad, ~] = cart2sph(relVec(1), relVec(2), relVec(3));
        calcAz = rad2deg(calcAzRad);
        calcEl = rad2deg(calcElRad);
        
        % 실제 측정 각도와 계산된 각도의 차이 (잔차)
        % Azimuth: 360도 환상성을 고려한 각도 차이 계산 (angdiff 사용)
        azDiff = rad2deg(angdiff(deg2rad(measuredAngles(i, 1)), deg2rad(calcAz)));
        elDiff = measuredAngles(i, 2) - calcEl;
        
        res(2*i-1) = azDiff;
        res(2*i)   = elDiff;
    end
end