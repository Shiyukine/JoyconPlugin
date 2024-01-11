using System;

namespace BetterJoyForCemu.Controller {
	public enum DpadDirection {
		None,
		Northwest,
		West,
		Southwest,
		South,
		Southeast,
		East,
		Northeast,
		North,
	}

	public struct OutputControllerDualShock4InputState {
		public bool triangle;
		public bool circle;
		public bool cross;
		public bool square;

		public bool trigger_left;
		public bool trigger_right;

		public bool shoulder_left;
		public bool shoulder_right;

		public bool options;
		public bool share;
		public bool ps;
		public bool touchpad;

		public bool thumb_left;
		public bool thumb_right;

		public DpadDirection dPad;

		public byte thumb_left_x;
		public byte thumb_left_y;
		public byte thumb_right_x;
		public byte thumb_right_y;

		public byte trigger_left_value;
		public byte trigger_right_value;

		public bool IsEqual(OutputControllerDualShock4InputState other) {
			bool buttons = triangle == other.triangle
				&& circle == other.circle
				&& cross == other.cross
				&& square == other.square
				&& trigger_left == other.trigger_left
				&& trigger_right == other.trigger_right
				&& shoulder_left == other.shoulder_left
				&& shoulder_right == other.shoulder_right
				&& options == other.options
				&& share == other.share
				&& ps == other.ps
				&& touchpad == other.touchpad
				&& thumb_left == other.thumb_left
				&& thumb_right == other.thumb_right
				&& dPad == other.dPad;

			bool axis = thumb_left_x == other.thumb_left_x
				&& thumb_left_y == other.thumb_left_y
				&& thumb_right_x == other.thumb_right_x
				&& thumb_right_y == other.thumb_right_y;

			bool triggers = trigger_left_value == other.trigger_left_value
				&& trigger_right_value == other.trigger_right_value;

			return buttons && axis && triggers;
		}
	}
}
