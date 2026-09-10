"""New-session live authority supervision; passive recorders are permitted."""
from threading import Lock


def _control_token(key):
    parts = key.split('/')
    if parts[:2] != ['tj', 'live']:
        return False
    return ((len(parts) == 5 and parts[2] == 'source') or
            (len(parts) == 6 and parts[2] in ('producer', 'executor') and parts[3] in ('arm', 'hand')) or
            (len(parts) == 6 and parts[2:4] == ['coordinator', 'arm']))


class LiveDomainGuard:
    def __init__(self, expected_tokens):
        if not expected_tokens or any(not isinstance(key, str) or not _control_token(key) for key in expected_tokens):
            raise ValueError('explicit canonical live control identities required')
        self.expected = frozenset(expected_tokens)
        self._lock = Lock()
        self._failure = None

    @property
    def failure(self):
        with self._lock:
            return self._failure

    def observe(self, key, *, present):
        if not isinstance(key, str) or type(present) is not bool:
            raise ValueError('invalid live token event')
        with self._lock:
            if self._failure:
                return
            if present and _control_token(key) and key not in self.expected:
                self._failure = 'conflicting live control authority: ' + key
            elif not present and key in self.expected:
                self._failure = 'required live authority lost: ' + key
