"""
Control System Filters and Utilities

Provides low-pass filters and helper functions for rover control system.

Classes:
    MovingAverageLPF: Simple moving average filter
    ExponentialMovingAverageLPF: Exponential moving average (EMA) filter

Functions:
    clamp: Saturate value to min/max bounds

Author: Vision Navigation System
Date: November 4, 2025
"""

from collections import deque
from typing import Deque, Optional
import numpy as np


class MovingAverageLPF:
    """
    Simple Moving Average Low-Pass Filter
    
    Computes average of last N samples for smoothing.
    """

    def __init__(self, window_size: int) -> None:
        """
        Initialize moving average filter.
        
        Args:
            window_size: Number of samples to average
        """
        self.window_size = window_size
        self.buffer: list = []

    def update(self, new_value: float) -> float:
        """
        Update filter with new sample and return filtered value.
        
        Args:
            new_value: New measurement value
            
        Returns:
            Filtered value (or input if buffer not full)
        """
        self.buffer.append(new_value)

        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)

        if len(self.buffer) == self.window_size:
            return sum(self.buffer) / self.window_size
        else:
            return new_value


class ExponentialMovingAverageLPF:
    """
    Exponential Moving Average (EMA) Low-Pass Filter
    
    Gives exponential weight to recent samples with configurable smoothing.
    Uses bounded history buffer to prevent memory growth.
    """

    def __init__(self, alpha: float, maxlen: int = 30) -> None:
        """
        Initialize EMA filter.
        
        Args:
            alpha: Smoothing factor (0 < alpha <= 1)
                   Higher alpha = more responsive to new data
                   Lower alpha = more smoothing
            maxlen: Maximum history buffer length
        """
        self.alpha = alpha
        self.ema: Optional[float] = None
        self.maxlen = maxlen
        self.buffer: Deque[float] = deque(maxlen=maxlen)

    def update(self, new_value: float) -> float:
        """
        Update filter with new sample and return EMA value.
        
        Args:
            new_value: New measurement value
            
        Returns:
            Exponential moving average
        """
        self.buffer.append(new_value)

        if self.ema is None:
            self.ema = new_value
        else:
            self.ema = self.alpha * new_value + (1 - self.alpha) * self.ema

        return self.ema

    def is_full(self) -> bool:
        """
        Check if history buffer is full (warmed up).
        
        Returns:
            True if buffer has reached maxlen samples
        """
        return len(self.buffer) >= self.maxlen

    def get_buffer_size(self) -> int:
        """Get current buffer fill level."""
        return len(self.buffer)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Saturate value to [min_val, max_val] range.
    
    Args:
        value: Value to saturate
        min_val: Minimum boundary
        max_val: Maximum boundary
        
    Returns:
        Saturated value
    """
    return max(min_val, min(value, max_val))


def pid_controller(
    error: float,
    kp: float,
    ki: float,
    kd: float,
    integral: float,
    last_error: float,
    dt: float
) -> tuple:
    """
    PID controller calculation.
    
    Args:
        error: Current error
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        integral: Accumulated integral term
        last_error: Previous error value
        dt: Time delta (seconds)
        
    Returns:
        Tuple: (control_output, new_integral, new_last_error)
    """
    # Integral term
    integral += error * dt
    integral = clamp(integral, -200, 200)

    # Derivative term
    d_term = (error - last_error) / dt if dt > 1e-6 else 0.0

    # Control output
    u = (kp * error) + (ki * integral) + (kd * d_term)

    return u, integral, error
