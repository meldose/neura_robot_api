from neura_ai_robot_api.deprecated_clients.edge_refiner_client import EdgeRefinerClient


# calling the main function
if __name__ == '__main__':
    detect_marker_client = EdgeRefinerClient()
    beg = [0, 0, 1, 0, 0, 0, 1]
    end = [0.1, 0, 1, 0, 0, 0, 1]
    print(detect_marker_client.refine_line(beg, end, False))
