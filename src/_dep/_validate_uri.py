"""Validates that a string represents a valid URI and then attempts to
ping it. If the URI is valid and pingable, the function returns True.
Setting keyword argument ping to False will skip the ping check."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import re
from typing import Optional

import requests
from vistutils.text import monoSpace


def _isValidURI(URI: str) -> bool:
  """
  Determine if the provided string is likely an URI.

  This function checks if the string follows common URI patterns,
  focusing on schemes and basic structure. It's not a comprehensive
  validation, but rather a quick likelihood check.

  Args:
  - s: The string to check.

  Returns:
  - True if the string likely represents an URI, False otherwise.
  """
  # Basic URI regex pattern
  uriPattern = re.compile(
    r'^(?:http|https)://'  # Common schemes
    r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+'  # Domain
    r'(?:[A-Z]{2,6}\.?|[A-Z0-9-]{2,}\.?)|'  # Domain extension
    r'localhost|'  # localhost
    r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
    r'(?::\d+)?'  # Optional port
    r'(?:/\S*)?$', re.IGNORECASE)  # Path
  return re.match(uriPattern, URI) is not None


def _isReachable(URI: str) -> bool:
  """
  Checks if the given URI is accessible by making a HTTP GET request.

  Args:
  - uri: The URI to check.

  Returns:
  - True if the resource is accessible, False otherwise.
  """
  try:
    response = requests.get(URI, timeout=5)  # 5 seconds timeout
    # Consider 200-400 status codes as success to account for redirects,
    # unauthorized, etc., which still indicate the URI is reachable.
    return 200 <= response.status_code < 400
  except requests.RequestException:
    return False


def validateURI(URI: str, **kwargs) -> Optional[str]:
  """Validates that a string represents a valid URI and then attempts to
  ping it. If the URI is valid and pingable, the function returns True.
  Setting keyword argument ping to False will skip the ping check."""
  if _isValidURI(URI):
    if kwargs.get('ping', True):
      if _isReachable(URI):
        return URI
      e = """The provided URI: '%s' is valid, but not reachable!"""
      raise ConnectionError(monoSpace(e % URI))
    return URI
  if kwargs.get('strict', True):
    e = """The provided string: '%s' does not represent a valid URI!"""
    raise ValueError(monoSpace(e % URI))
