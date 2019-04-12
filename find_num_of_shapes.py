import cv2 as cv


def preprocessing(image):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    gray = cv.medianBlur(gray, 5)
    ret, thresh = cv.threshold(gray, 127, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    return thresh


def find_white_board(image):
    thresh = preprocessing(image)
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    max_area = 0
    max_cnt = None
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_cnt = cnt

    if max_cnt is None:
        return image

    x, y, w, h = cv.boundingRect(max_cnt)
    return image[y:y+h, x:x+w]


def find_main_white_board(image):
    thresh = preprocessing(image)
    thresh = 255 - thresh
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    image_h, image_w, _ = image.shape
    bounding = 0.01
    bounding_x = image_w * bounding
    bounding_y = image_h * bounding

    target_contours = []
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > 100:
            x, y, w, h = cv.boundingRect(cnt)
            if x > bounding_x and x + w < image_w - bounding_x and \
                y > bounding_y and y + h < image_h - bounding_y:
                target_contours.append(cnt)

    min_x = image_w
    min_y = image_h
    max_x = 0
    max_y = 0
    for cnt in target_contours:
        x, y, w, h = cv.boundingRect(cnt)
        if x + w > max_x:
            max_x = x + w
        if x < min_x:
            min_x = x
        if y + h > max_y:
            max_y = y + h
        if y < min_y:
            min_y = y

    space = 5
    min_y = max(min_y - space, 0)
    max_y = min(max_y + space, image_h)
    min_x = max(min_x - space, 0)
    max_x = min(max_x + space, image_w)
    return image[min_y:max_y, min_x:max_x]


def find_num_of_shapes(image):
    num_of_shapes = {
        "circle": 0,
        "line": 0,
        "triangle": 0,
        "rectangle": 0
    }
    thresh = preprocessing(image)
    thresh = 255 - thresh
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > 100:
            cv.drawContours(image, [cnt], 0, (0, 255, 0), 2)

            x, y, w, h = cv.boundingRect(cnt)
            epsilon = 0.03 * cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, epsilon, True)
            vertex = len(approx)
            cv.putText(image, str(vertex), (x + int(w / 2) - 5, y + int(h / 2) + 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), lineType=cv.LINE_AA)

            if vertex == 2:
                num_of_shapes["line"] += 1
            elif vertex == 3:
                num_of_shapes["triangle"] += 1
            elif vertex == 4:
                num_of_shapes["rectangle"] += 1
            else:
                num_of_shapes["circle"] += 1

    return image, num_of_shapes


for i in range(1, 5):
    img = cv.imread("images/" + str(i) + ".jpg", cv.IMREAD_COLOR)
    img = cv.resize(img, (int(img.shape[1] / 2), int(img.shape[0] / 2)))

    board = find_white_board(img)
    board = find_main_white_board(board)
    board, num_of_shapes = find_num_of_shapes(board)
    print(num_of_shapes)

    cv.imshow("image", img)

    cv.waitKey(0)
