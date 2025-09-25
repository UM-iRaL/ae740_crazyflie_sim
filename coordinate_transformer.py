#!/usr/bin/env python3
"""
Standalone Coordinate Transformation Utility
============================================

This module provides a simple function to transform real-world coordinates 
to image coordinates using a pre-computed transformation matrix.

Usage:
    1. Load your transformation matrix from the saved file
    2. Call transform_coordinates() to convert points
    
Example:
    import numpy as np
    
    # Load the transformation
    data = np.load('coordinate_mapping.npz')
    transform_matrix = data['transform_matrix']
    
    # Transform a point
    real_world_point = (-0.5, 0.5)  # (x, y) in meters
    image_coords = transform_coordinates(real_world_point, transform_matrix)
    print(f"Real-world {real_world_point} -> Image {image_coords}")
"""

import numpy as np


def transform_coordinates(real_world_points, transform_matrix):
    """
    Transform real-world coordinates to image pixel coordinates.
    
    This function applies the pre-computed transformation matrix to convert
    real-world (x, y) coordinates in meters to image (x, y) coordinates in pixels.
    
    Parameters:
    -----------
    real_world_points : tuple, list, or np.array
        Real-world coordinates to transform. Can be:
        - Single point: (x, y) or [x, y] 
        - Multiple points: [(x1, y1), (x2, y2), ...] or numpy array of shape (N, 2)
    
    transform_matrix : np.array
        2x3 transformation matrix from coordinate_mapping.npz
        Format: [[a, b, tx], [c, d, ty]] where:
        - (a, b, c, d) encode rotation and scaling
        - (tx, ty) encode translation
    
    Returns:
    --------
    np.array or tuple
        Transformed image coordinates in pixels:
        - Single point input: returns tuple (img_x, img_y) 
        - Multiple points input: returns numpy array of shape (N, 2)
    
    Examples:
    ---------
    # Transform single point
    >>> real_point = (-0.5, 0.5)
    >>> img_point = transform_coordinates(real_point, transform_matrix)
    >>> print(f"Image coordinates: ({img_point[0]:.1f}, {img_point[1]:.1f})")
    
    # Transform multiple points
    >>> real_points = [(-0.5, 0.5), (0.0, 0.0), (0.5, -0.5)]
    >>> img_points = transform_coordinates(real_points, transform_matrix)
    >>> for i, (real, img) in enumerate(zip(real_points, img_points)):
    >>>     print(f"Point {i}: {real} -> ({img[0]:.1f}, {img[1]:.1f})")
    """
    
    # Handle input format
    points = np.array(real_world_points, dtype=np.float32)
    single_point = False
    
    # Check if single point was provided
    if points.ndim == 1 and len(points) == 2:
        points = points.reshape(1, -1)
        single_point = True
    elif points.ndim == 2 and points.shape[1] != 2:
        raise ValueError("Points must have 2 coordinates (x, y). Got shape: {}".format(points.shape))
    
    # Validate transformation matrix
    if transform_matrix.shape != (2, 3):
        raise ValueError("Transform matrix must be 2x3. Got shape: {}".format(transform_matrix.shape))
    
    # Apply 2D affine transformation
    # For each point [x, y], compute: [x', y'] = [[a, b, tx], [c, d, ty]] * [x, y, 1]
    # This becomes: x' = a*x + b*y + tx, y' = c*x + d*y + ty
    
    # Add homogeneous coordinate (column of ones)
    ones = np.ones((points.shape[0], 1), dtype=np.float32)
    homogeneous_points = np.hstack([points, ones])  # Shape: (N, 3)
    
    # Apply transformation: (2, 3) @ (N, 3).T = (2, N) -> (N, 2)
    transformed = (transform_matrix @ homogeneous_points.T).T
    
    # Return format based on input
    if single_point:
        return tuple(transformed[0])
    else:
        return transformed


def load_transformation_matrix(filepath='coordinate_mapping.npz'):
    """
    Convenience function to load the transformation matrix.
    
    Parameters:
    -----------
    filepath : str
        Path to the .npz file containing the transformation matrix
        
    Returns:
    --------
    dict
        Dictionary containing:
        - 'transform_matrix': 2x3 transformation matrix
        - 'time_offset': time synchronization offset (seconds)
        - 'rmse': transformation quality metric (pixels)
    """
    try:
        data = np.load(filepath)
        return {
            'transform_matrix': data['transform_matrix'],
            'time_offset': data['time_offset'],
            'rmse': data['rmse']
        }
    except FileNotFoundError:
        raise FileNotFoundError(f"Transformation file not found: {filepath}")
    except KeyError as e:
        raise KeyError(f"Missing key in transformation file: {e}")


def validate_image_bounds(image_coords, max_width=4000, max_height=3000):
    """
    Check if transformed coordinates are within reasonable image bounds.
    
    Parameters:
    -----------
    image_coords : tuple or np.array
        Image coordinates to validate
    max_width, max_height : int
        Maximum expected image dimensions
        
    Returns:
    --------
    bool or np.array
        True if coordinates are within bounds, False otherwise
    """
    coords = np.array(image_coords)
    
    if coords.ndim == 1:  # Single point
        return (0 <= coords[0] <= max_width) and (0 <= coords[1] <= max_height)
    else:  # Multiple points
        return ((0 <= coords[:, 0]) & (coords[:, 0] <= max_width) & 
                (0 <= coords[:, 1]) & (coords[:, 1] <= max_height))


# Example usage and testing
if __name__ == "__main__":
    print("Coordinate Transformation Utility")
    print("=" * 50)
    
    try:
        # Load transformation
        mapping_data = load_transformation_matrix('coordinate_mapping.npz')
        transform_matrix = mapping_data['transform_matrix']
        
        print(f"Loaded transformation matrix:")
        print(f"RMSE: {mapping_data['rmse']:.2f} pixels")
        print(f"Time offset: {mapping_data['time_offset']:.4f} seconds")
        print()
        
        # Test single point transformation
        test_point = (-0.5, 0.5)
        img_coords = transform_coordinates(test_point, transform_matrix)
        is_valid = validate_image_bounds(img_coords)
        
        print(f"Single point test:")
        print(f"Real-world: ({test_point[0]:.1f}, {test_point[1]:.1f})")
        print(f"Image:      ({img_coords[0]:.1f}, {img_coords[1]:.1f})")
        print(f"Valid:      {'✓' if is_valid else '✗'}")
        print()
        
        # Test multiple points transformation
        test_points = [(-1.0, 0.5), (-0.5, 0.0), (0.0, 0.5), (0.5, 0.0)]
        img_points = transform_coordinates(test_points, transform_matrix)
        valid_flags = validate_image_bounds(img_points)
        
        print(f"Multiple points test:")
        for i, (real, img, valid) in enumerate(zip(test_points, img_points, valid_flags)):
            print(f"  Point {i+1}: ({real[0]:5.1f}, {real[1]:5.1f}) -> ({img[0]:7.1f}, {img[1]:7.1f}) {'✓' if valid else '✗'}")
            
    except FileNotFoundError:
        print("coordinate_mapping.npz not found. Run the mapping estimation tool first.")
    except Exception as e:
        print(f"Error: {e}")


# ============================================================================
# COPY-PASTE FUNCTION - Use this in your other files
# ============================================================================

def real_to_image_transform(real_world_points, transform_matrix):
    """
    COPY-PASTE FUNCTION: Transform real-world coordinates to image coordinates
    
    Copy this entire function to any Python file where you need coordinate transformation.
    
    Usage:
        # Load transformation matrix (do this once)
        data = np.load('coordinate_mapping.npz')  # or your path
        transform_matrix = data['transform_matrix']
        
        # Transform coordinates (use as needed)
        real_point = (-0.5, 0.5)  # meters
        img_point = real_to_image_transform(real_point, transform_matrix)
        print(f"Real {real_point} -> Image ({img_point[0]:.1f}, {img_point[1]:.1f})")
    
    Parameters:
    -----------
    real_world_points : tuple or list
        (x, y) coordinates in real-world units (meters)
    transform_matrix : numpy array  
        2x3 transformation matrix from coordinate_mapping.npz
        
    Returns:
    --------
    tuple : (img_x, img_y) coordinates in pixels
    """
    import numpy as np
    
    # Convert to numpy array and ensure proper format
    point = np.array(real_world_points, dtype=np.float32).reshape(1, 2)
    
    # Add homogeneous coordinate and apply transformation
    homogeneous = np.hstack([point, np.ones((1, 1), dtype=np.float32)])
    transformed = (transform_matrix @ homogeneous.T).T
    
    return float(transformed[0, 0]), float(transformed[0, 1])