"""Tests for src.label_store — YOLO label file parsing/serialization."""

from src.label_store import LabelRecord, LabelStore, _parse_line


class TestParseLine:
    def test_detection_line(self):
        record = _parse_line("2 0.5 0.5 0.2 0.3")
        assert record == LabelRecord(class_id=2, coords=[0.5, 0.5, 0.2, 0.3])

    def test_segmentation_line_with_many_coords(self):
        record = _parse_line("0 0.1 0.1 0.2 0.2 0.3 0.1")
        assert record == LabelRecord(class_id=0, coords=[0.1, 0.1, 0.2, 0.2, 0.3, 0.1])

    def test_empty_line_returns_none(self):
        assert _parse_line("") is None

    def test_whitespace_only_line_returns_none(self):
        assert _parse_line("   \t  ") is None

    def test_non_numeric_class_id_returns_none(self):
        assert _parse_line("not_a_number 0.5 0.5") is None

    def test_non_numeric_coordinate_returns_none(self):
        assert _parse_line("0 0.5 not_a_float") is None

    def test_class_id_with_no_coords_is_valid(self):
        # A class with an empty coords list is syntactically valid per the format.
        record = _parse_line("5")
        assert record == LabelRecord(class_id=5, coords=[])

    def test_extra_whitespace_is_tolerated(self):
        record = _parse_line("  1   0.25   0.75  ")
        assert record == LabelRecord(class_id=1, coords=[0.25, 0.75])


class TestLabelStoreSaveAndLoadRaw:
    def test_save_then_load_raw_round_trips(self, tmp_path):
        store = LabelStore(cache=None)
        path = tmp_path / "labels.txt"
        records = [
            LabelRecord(class_id=0, coords=[0.5, 0.5, 0.2, 0.3]),
            LabelRecord(class_id=1, coords=[0.1, 0.1, 0.9, 0.9]),
        ]

        store.save(path, records)
        loaded = store.load_raw(path)

        assert loaded == records

    def test_load_raw_on_missing_file_returns_empty_list(self, tmp_path):
        store = LabelStore(cache=None)
        assert store.load_raw(tmp_path / "does_not_exist.txt") == []

    def test_save_formats_coords_to_six_decimal_places(self, tmp_path):
        store = LabelStore(cache=None)
        path = tmp_path / "labels.txt"
        store.save(path, [LabelRecord(class_id=3, coords=[1 / 3])])

        content = path.read_text(encoding="utf-8")
        assert content == "3 0.333333"

    def test_load_raw_skips_malformed_lines(self, tmp_path):
        store = LabelStore(cache=None)
        path = tmp_path / "labels.txt"
        path.write_text("0 0.1 0.1\nnot valid\n1 0.2 0.2\n", encoding="utf-8")

        loaded = store.load_raw(path)

        assert loaded == [
            LabelRecord(class_id=0, coords=[0.1, 0.1]),
            LabelRecord(class_id=1, coords=[0.2, 0.2]),
        ]
