# Line_Detection_Project
차선인식 프로젝트

<aside>
🚗 팀명 : 조수석
	
팀원 : 김민석 , 조남훈

발표자 : 조남훈

</aside>

## 프로젝트 목표

<aside>
💡 주어진 영상 좌, 우 양쪽의 차선을 최대한 정확하게 인식하여 lpos , rpos 좌표 구하기

- `y == 400` 지점에서 `lpos`, `rpos` 추출

![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/f5f310b9-5399-4235-a5c2-c5aac76314da)


</aside>

# 📓목차

### [1️⃣ 엣지 검출]

### [2️⃣ lpos와 rpos의 좌표 구하기]

### [3️⃣구한 좌표 위에 그림 그리기]

### [4️⃣문제점 및 개선 사항]

### [5️⃣최종 결과]

### [6️⃣향후 작업 및 소감(선택)]

---

# 1️⃣ 엣지 검출

- `getLane` 함수 코드
    
    ```cpp
    Mat getLane(const Mat& frame, const Mat& roi_mask)
    {	
    	Mat edge;
    	Canny(frame, edge, 195, 255);
    
    	Mat blur;
    	GaussianBlur(edge, blur, Size(3, 3), 0);
    
    	Mat roi;
    	blur.copyTo(roi, roi_mask);
    	rectangle(roi, Rect(228, 397, 188, 83), Scalar(0), -1); // lidar_mask
    
    	vector<Vec4i> lines;
    	HoughLinesP(roi, lines, 1, CV_PI / 180, 1, 50, 5);
    
    	// Draw lines
    	for (size_t i = 0; i < lines.size(); i++) {
    		Vec4i l = lines[i];
    		line(roi, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
    	}
    	
    	return roi;
    }
    ```
    
    ---
    

- `ROI Mask`

```cpp
// Trapezoid ROI
vector<Point> roi_pts(4);
roi_pts[0] = Point(0, 480); // bottom left
roi_pts[1] = Point(20, 350); // top left
roi_pts[2] = Point(620, 350); // top right
roi_pts[3] = Point(640, 480); // bottom right
```

- 사다리꼴 ROI를 사용함으로써 노이즈 직선들 최대한 제거

- `ROI Mask` 영상

![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/e3c641b2-ac7a-4d57-939e-af98bf92cbd1)


---

- `Lidar Mask`

```cpp
rectangle(roi, Rect(228, 397, 188, 83), Scalar(0), -1); // lidar_mask
```

- `Lidar Mask` 영상

![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/d187b662-4047-4158-a035-491757fc5a1f)


---

- `Canny`후 `GaussianBlur`적용한 영상
    
    ![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/3c397178-3f86-4f14-81c8-7fc86f7bf716)

    

- `Canny`만 적용한 영상
    
    ![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/0c0de80d-7bbd-478e-bb70-be82f5cfb911)

    

---

<aside>
💡 `Canny` 후에 `GaussianBlur` 이유 ?

- `Canny`에도 `Gaussian`이 포함되어 있지만 `Canny`만 실행했을 때 출력 영상이 샤프하다고 판단
    - 엣지 부분이 얇고 직선 검출이 뚝뚝 끊기거나 잘 안된다고 판단
- 이 현상을 해결하기 위해 `Sobel`, `Morphology` , `Gaussian Blur`를 고민
- `Sobel`와 `Morphology`을 했을 시 직선이 넓어지긴 했지만 부드럽다기보다는 울퉁불퉁해짐
- `Gaussian Blur`를 사용하여 부드러운 직선 검출
</aside>

- 직선이 지저분하더라도 검출만 되면 OK!

- 연산 속도를 위해 그레이스케일 영상 그대로 직선 표시(`Scalar(255)` )

# 2️⃣ lpos와 rpos의 좌표 구하기

- `getPos` 함수 코드

```cpp
vector<int>getPos(const Mat& roi)
{
	const uchar* offset = roi.ptr<uchar>(400);
	int cols = roi.cols;
	int half_cols = roi.cols / 2;
	
	//lpos
	vector<int> lpos;
	for (int i = 10; i < half_cols; ++i) {
		if (offset[i] == 255) {
			lpos.push_back(i);
		}
	}

	int lpos_size = lpos.size();
	int lpos_x = lpos_size > 0 ? (lpos[0] + lpos[lpos_size - 1]) / 2 : 0; // (lposl + lposr) / 2
	
	//rpos
	vector<int> rpos;
	for (int n = half_cols; n < cols; ++n) {
		if (offset[n] == 255) {
			rpos.push_back(n);
		}
	}

	int rpos_size = rpos.size();
	int rpos_x = rpos_size > 0 ? (rpos[0] + rpos[rpos_size - 1]) / 2 : cols; // (rposl + rposr) / 2

	return {lpos_x , rpos_x};
}
```

<aside>
💡 `lpos`와 `rpos` 좌표를 어떤 방법으로 구할 것 인가?

 → 빠른 행접근을 위해 `ptr` 을 사용하여 400번째 행(offset)의 픽셀 값들을 가져온다.

 → `ROI`영상을 세로 기준 절반으로 나눠 왼쪽 영역은 `lpos`  오른쪽 영역은 `rpos` 좌표를 

구한다.

 → 400번째 행의 있는 픽셀 중 `Scalar(255)` 인 값만 벡터에 저장한다.

 → 벡터의 첫 번째 값은 `lposl`, 마지막 값은 `lposr` 이 된다.

 → `lposl` 과 `lposr` 을 더한 후 2로 나눠 `lpos` 의 x좌표를 구한다. (평균 값 계산)

</aside>

- 평균 값 계산을 했을 때 장점?
    
    → 1. 검출한 직선의 상태에 상관 없이 일정한 `lpos` 값을 얻을 수 있다.
    
    → 2. 예를 들어, `lpos`보다 `노이즈 엣지`가 더 왼쪽에 있는 경우 적절한 `Roi`로 인해 `lposl` 값과 큰 차이가 발생하지 않을 것
    
    → 3. 평균 값 계산으로 인해 `lpos` 좌표가 차선 안쪽에서 움직이게 된다.
    

![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/856c763e-f719-471c-b708-fd250fdfd5e0)


![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/cd37e5d8-b166-40a7-9768-9535738d8d0b)


# 3️⃣구한 좌표 위에 그림 그리기

- `drawonFrame` 함수 코드

```cpp
void drawonFrame(Mat& frame , vector<int>pos_coord)
{
	line(frame, Point(0, 400), Point(640, 400), Scalar(0, 255, 0), 1, LINE_AA);
	
	for (int x : pos_coord) {
		string text = "x : " + to_string(x) + " y : " + "400";
		circle(frame, Point(x, 400), 4, Scalar(0, 0, 255), 2, -1);
		putText(frame, text, Point(x, 380), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255,0,0), 1);
	}
}
```

![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/8c6b06d7-723c-4763-b70c-bd2a292520e6)


# 4️⃣문제점 및 개선 사항

- 코너부분에서 차선 검출이 잘 되지 않는 구간 발생

![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/f2a651d5-860b-461d-8b8b-ec2613e20f16)


→ `int lpos_x = lpos_size > 0 ? (lpos[0] + lpos[lpos_size - 1]) / 2 : 0;`

→ `lpos` 가 검출 되지 않았을 시 그 값을 0 이 아니라 `Sobel`필터를 통해 한번 더 검출을 시도한다.

→  그럼에도 검출이 안되었을 시엔 차선이 없는 구간으로 인식!

- `Roi` 에 대한 의존성
    
    → 현재 영상에서는 `Roi`가 적절하다고 느끼지만 `Roi`가 커지거나 변경되면 많은 노이즈 직선들이 발생할 확률이 높음
    
    → 노이즈 발생 시 평균 값 계산을 하더라도 `lpos` 값이 크게 튈 수 있다.
    

---

# 5️⃣최종 결과

- 출력 영상
    
    https://youtu.be/8pc017com_Q

- csv 파일 및 정확도 확인 (정답과 비교)
    
    ![image](https://github.com/Chonamhoon/Line_Detection_Project/assets/145315696/f89fdab9-b107-4de1-bea5-edb8d57fdf1a)

    

## 👬협업 과정

- 소스 코드 작성 : `Github`를 사용
    - 각자 `branch`생성해 개발 후 `push`
    - 매일 정해진 시간에 `허들` 진행
        - 10.31 저녁 : 협업 방식 결정
        - 11.1 점심(13:00) : 1차 코드 리뷰
        - 11.1 저녁 (19:00): 2차 코드 리뷰(중간 제출용)
        - 11.2 저녁 (19:00): 3차 코드 리뷰 (최종 제출용)
    - 이외 진전 있을 시 `Slack`, `허들`을 통해 상황 공유

---

# 6️⃣향후 작업 및 소감(선택)

프로젝트가 끝난 이후에도 더 작업을 진행한다면 어떤 부분들을 할 예정인가요?

- 김민석
    - 배운 것들을 직접 적용할 좋은 기회였지만, 생각대로 잘 되지 않아 아쉬움이 크다.
        - 가장 아쉬웠던 것은 `inRange`함수를 통한 이진화로 노이즈를 최소화 시키고 싶었는데, 해당 부분이 잘 되지 않았던 점
    - 실제로 사용되는 검정색 아스팔트, 흰색 차선 & 노란색 차선을 대상으로 진행해보고 싶다.
    - 또 낮, 밤, 비, 안개 등 다양한 상황에서 차선 인식을 진행해 보고 싶다.

- 조남훈
    - LMS영상에서의 실습할 때 와는 달리 직접 코드를 짤 때 많이 부족하다고 느낌
        - 어떠한 필터와 어떠한 처리를 해야할지 고민하는 부분들이 너무 많았다.
    - 추후에 소벨 필터나 칼만 필터를 사용하여 코너 구간에서도 직선 검출이 잘 이루어 지도록 개선 하고 싶다.
