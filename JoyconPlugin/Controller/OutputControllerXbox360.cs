namespace BetterJoyForCemu.Controller {
	public struct OutputControllerXbox360InputState {
		// buttons
		public bool thumb_stick_left;
		public bool thumb_stick_right;

		public bool y;
		public bool x;
		public bool b;
		public bool a;

		public bool start;
		public bool back;

		public bool guide;

		public bool shoulder_left;
		public bool shoulder_right;

		// dpad
		public bool dpad_up;
		public bool dpad_right;
		public bool dpad_down;
		public bool dpad_left;

		// axis
		public short axis_left_x;
		public short axis_left_y;

		public short axis_right_x;
		public short axis_right_y;

		// triggers
		public byte trigger_left;
		public byte trigger_right;

		public bool IsEqual(OutputControllerXbox360InputState other) {
			bool buttons = thumb_stick_left == other.thumb_stick_left
				&& thumb_stick_right == other.thumb_stick_right
				&& y == other.y
				&& x == other.x
				&& b == other.b
				&& a == other.a
				&& start == other.start
				&& back == other.back
				&& guide == other.guide
				&& shoulder_left == other.shoulder_left
				&& shoulder_right == other.shoulder_right;

			bool dpad = dpad_up == other.dpad_up
				&& dpad_right == other.dpad_right
				&& dpad_down == other.dpad_down
				&& dpad_left == other.dpad_left;

			bool axis = axis_left_x == other.axis_left_x
				&& axis_left_y == other.axis_left_y
				&& axis_right_x == other.axis_right_x
				&& axis_right_y == other.axis_right_y;

			bool triggers = trigger_left == other.trigger_left
				&& trigger_right == other.trigger_right;

			return buttons && dpad && axis && triggers;
		}
	}
}
