def clip(value: float, min_value: float, max_value: float) -> float:
    """
    Clips a value to a specified range.

    Parameters
    ----------
    `value` : float
        The value to be clipped.
    `min_value` : float
        The minimum value to clip to.
    `max_value` : float
        The maximum value to clip to.

    Returns
    -------
    float
        The clipped value.
    """
    lower_limited_value = max(value, min_value)
    lower_and_upper_limited_value = min(lower_limited_value, max_value)
    return lower_and_upper_limited_value
