"""
In the simulation, a reference object was placed in the scene to help in the conversion of pixel coordinates to world coordinates. The reference object was a cube with the following dimensions:
Cube_Dimensions = [0.06, 0.06, 0.06]
All the posiitons should be in robot's coordinates, so if a target position had more than one digits it is not in the robot's coordinates. 
"""


def pixel_to_sim(px, py):
    # Reference points in simulation space
    # these references are from the simulation and object detection from the system VLM output
    sim_x1, sim_y1 = 0.0502, 0.6592
    sim_x2, sim_y2 = 0.3928, 0.7176

    # Corresponding points in pixel space
    pixel_x1, pixel_y1 = 332, 127
    pixel_x2, pixel_y2 = 475, 96

    # Compute scale
    scale_x = (sim_x2 - sim_x1) / (pixel_x2 - pixel_x1)
    scale_y = (sim_y2 - sim_y1) / (pixel_y2 - pixel_y1)

    # Compute offset
    offset_x = sim_x1 - pixel_x1 * scale_x
    offset_y = sim_y1 - pixel_y1 * scale_y

    # Convert pixel to sim
    sim_x = px * scale_x + offset_x
    sim_y = py * scale_y + offset_y

    sim_x = round(sim_x, 4)
    sim_y = round(sim_y, 4)

    return sim_x, sim_y