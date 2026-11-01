"""Tests for attrs field converters on BringupMeta."""

from __future__ import annotations


class TestBringupMetaRequires:
    def test_string_wraps_to_singleton_not_chars(self):
        """Critical: bare string must NOT splat to character set."""
        from arena_robots.bringup import BringupMeta

        meta = BringupMeta(requires="mobile")
        assert meta.requires == frozenset({"mobile"})

    def test_list_input_coerces_to_frozenset(self):
        from arena_robots.bringup import BringupMeta

        meta = BringupMeta(requires=["mobile", "arm"])
        assert meta.requires == frozenset({"mobile", "arm"})

    def test_set_input_coerces_to_frozenset(self):
        from arena_robots.bringup import BringupMeta

        meta = BringupMeta(requires={"mobile"})
        assert meta.requires == frozenset({"mobile"})

    def test_tuple_input_coerces_to_frozenset(self):
        from arena_robots.bringup import BringupMeta

        meta = BringupMeta(requires=("mobile", "arm"))
        assert meta.requires == frozenset({"mobile", "arm"})

    def test_frozenset_input_is_idempotent(self):
        from arena_robots.bringup import BringupMeta

        original = frozenset({"mobile"})
        meta = BringupMeta(requires=original)
        assert meta.requires == original

    def test_empty_iterable(self):
        from arena_robots.bringup import BringupMeta

        meta = BringupMeta(requires=[])
        assert meta.requires == frozenset()
