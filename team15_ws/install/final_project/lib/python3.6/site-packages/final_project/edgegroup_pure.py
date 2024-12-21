import cv2
import numpy as np
import csv
import os
from final_project.homography import solve_homography

class EdgeGroup:
    def __init__(self, min_path = 10, radius=5, use_canny_edge = False, closing = False, circle=False, show_term=False, img_save_path="./edge_imgs"):
        self.img_save_path = img_save_path
        # create save directory if not exists
        os.makedirs(self.img_save_path, exist_ok=True)
        self.show_term=show_term
        self.min_path = min_path
        if circle:
            self.cover_mask, self.next_mask, self.real_mask = self.generate_circle_mask(radius)
        else:
            self.cover_mask, self.next_mask, self.real_mask = self.generate_square_mask(radius)

        self.use_canny_edge = use_canny_edge
        self.closing = closing
        return
    
    def generate_circle_mask(self, radius):
        covered_points = []
        edge_points = []
        edge_mask = np.zeros((radius*2+1, radius*2+1), dtype=np.uint8)
        real_mask = np.zeros((radius*2+1, radius*2+1), dtype=np.uint8)
        for x in range(-radius, radius + 1):
            for y in range(-radius, radius + 1):
                distance_squared = x**2 + y**2
                if abs(distance_squared - radius**2) <= 4:
                    edge_points.append((x, y))
                    edge_mask[y+radius, x+radius] = 1
                elif distance_squared <= radius**2:
                    covered_points.append((x, y))
                    real_mask[y+radius, x+radius] = 1
        if self.show_term:
            cv2.imwrite(self.img_save_path + "/cover_mask.png", real_mask*255)
            cv2.imwrite(self.img_save_path + "/edge_mask.png", edge_mask*255)
        return covered_points, edge_points, real_mask
    
    def generate_square_mask(self, radius):
        covered_points = []
        edge_points = []
        edge_mask = np.zeros((radius*2+1, radius*2+1), dtype=np.uint8)
        real_mask = np.zeros((radius*2+1, radius*2+1), dtype=np.uint8)
        for x in range(-radius, radius + 1):
            for y in range(-radius, radius + 1):
                distance_squared = max(abs(x), abs(y))
                if distance_squared == radius:
                    edge_points.append((x, y))
                    edge_mask[y+radius, x+radius] = 1
                elif distance_squared < radius:
                    covered_points.append((x, y))
                    real_mask[y+radius, x+radius] = 1
        if self.show_term:
            cv2.imwrite(self.img_save_path + "/cover_mask.png", real_mask*255)
            cv2.imwrite(self.img_save_path + "/edge_mask.png", edge_mask*255)
        return covered_points, edge_points, real_mask

    def generate_path(self, image_path, csv_path, target_corners):
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        self.img_height, self.img_width = img.shape
        ref_corners = np.array([[0, 0], [self.img_width, 0], [self.img_width, self.img_height], [0, self.img_height]])
        H = solve_homography(ref_corners, target_corners)
        if self.use_canny_edge:
            edges = cv2.Canny(img, 100, 200)
        else:
            _, edges = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
        if self.show_term:
            cv2.imwrite(self.img_save_path + "/drawing.png", edges)
        if self.closing:
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, self.real_mask)
            if self.show_term:
                cv2.imwrite(self.img_save_path + "/closing.png", edges)
        pathes = self.find_continuous_paths(edges)
        if True:
            pathes = self.transfer_H(H, 0, 0, pathes)
        group_count = self.write_csv(pathes, csv_path)
        if self.show_term:
            print("(group, path, point):", group_count)
        if self.show_term:
            self.output_covered(pathes, self.img_save_path + "/test.png", (np.max(target_corners,axis=0)[1], np.max(target_corners,axis=0)[0]) if True else img.shape)
        return
    
    def find_continuous_paths(self, edges):
        visited = np.zeros(edges.shape, dtype=np.uint8)
        rows = edges.shape[0]
        cols = edges.shape[1]
        to_visit = []
        groups = []
        for y in range(rows):
            for x in range(cols):
                if(edges[y, x] > 0 and not visited[y, x]):
                    to_visit.append((x, y))
                    if(not groups or len(groups[-1]) > 0):
                        groups.append([])
                    while(to_visit):
                        current = to_visit.pop()
                        if(visited[current[1], current[0]]):
                            continue
                        path = self.trace_path(edges, visited, current, to_visit)
                        if(len(path) >= self.min_path):
                            groups[-1].append(path)
        return groups
    
    def trace_path(self, edges, visited, start, to_visit):
        path = []
        visited[start[1], start[0]] = 1
        path.append(start)
        current = (start[0], start[1])
        path_complete = False
        moment = (0, 0)
        while(not path_complete):
            path_complete = True
            moment_dist = -1
            for ax, ay in self.next_mask:
                nx = current[0] + ax
                ny = current[1] + ay
                if(nx >= 0 and ny >= 0 and nx < edges.shape[1] and ny < edges.shape[0]):
                    if(edges[ny, nx] > 0 and not visited[ny, nx]):
                        to_visit.append((nx, ny))
                        if moment_dist == -1 or moment_dist > abs(ax-moment[0])+abs(ay-moment[1]):
                            n_moment = (ax, ay)
                            moment_dist = abs(ax-moment[0])+abs(ay-moment[1])
            if moment_dist != -1:
                nx = current[0] + n_moment[0]
                ny = current[1] + n_moment[1]
                path.append((nx, ny))
                current = (nx, ny)
                path_complete = False
                moment = n_moment
            for ax, ay in self.cover_mask:
                nx = current[0] + ax
                ny = current[1] + ay
                if(nx >= 0 and ny >= 0 and nx < edges.shape[1] and ny < edges.shape[0]):
                    visited[ny, nx] = 1
        return path
    
    def output_covered(self, pathes, output_path, shape):
        cover = np.zeros(shape, dtype=np.uint8)
        for group in pathes:
            for path in group:
                for s_point, e_point in zip(path[:-1:], path[1::]):
                    cv2.line(cover, s_point, e_point, 255, 1)
        cv2.imwrite(output_path, cover)

    def transfer_H(self, H, H_corner_x, H_corner_y, pathes):
        rt = []
        for group in pathes:
            rt_group = []
            for path in group:
                rt_path = []
                for point in path:
                    np_point = np.array([H_corner_x+point[0], H_corner_y+point[1], 1.0])
                    np_point = np.dot(H, np_point)
                    np_point /= np_point[-1]
                    np_point = np_point[0:2].tolist()
                    np_point = (int(np_point[0]), int(np_point[1]))
                    rt_path.append(np_point)
                rt_group.append(rt_path)
            rt.append(rt_group)
        return rt
    
    def write_csv(self, pathes, csv_path):
        total_len = 0
        total_path = 0
        total_group = len(pathes)
        with open(csv_path, "w", newline="") as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=",", quotechar="\n")
            for group in pathes:
                total_path += len(group)
                for path in group:
                    total_len += len(path)
                    for point in path:
                        spamwriter.writerow(point)
                    spamwriter.writerow((-1, -1))
                spamwriter.writerow((-1, -1))
        return (total_group, total_path, total_len)

if __name__ == "__main__":
    app = EdgeGroup(use_canny_edge=False, closing=False, show_term=True, circle=False, radius=2, min_path=10)
    app.generate_path("./temp_files/simplified_strokes.png", "./temp_files/test.csv")
