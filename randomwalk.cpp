const float slp = 0.5;
const float reflect_distance = 6.0;
const float rotate_speed = 1.0;
const float forward_speed = 1.0;

class Randomwalk {   //this is the normal random walk
	Motor* left;
	Motor* right;
	IR* irlf;
	IR* irlb;
	IR* irf;
	IR* ir;
	mraa::Gpio* uirb;
	Location* start;
	Location* current;
	Odometry* odo;
	float current_angle;
	int channel;
	bool interuption;
	bool initialized;
	float in_angle; float mid_angle; float out_angle; bool wait; int cnt; int cntinc; bool dec;
public:
	Randomwalk(Motor* _left, Motor* _right, IR* _irlf, IR* _irlb, IR* _irf, IR* _ir, mraa::Gpio* _uirb, Location* _start) {
		left = _left;
		right = _right;
		irlf = _irlf;
		irlb = _irlb;
		ir = _ir;
		irf = _irf;
		uirb = _uirb;
		start = _start;
		current = _current;
		current_angle = start->theta();
		odo = new Odometry(left,right,start->x(),start->y(),start->theta());
	}
	void channel_stop() {
		left->stop();
		right->stop();
		initialized = false;
	}
	void forward_setup() {
		left->forward();
		right->forward();
		left->setSpeed(forward_speed);
		right->setSpeed(forward_speed);
	}
	void forward_run() {
		odo->run();
		left->run();
		right->run();
	}
	void reflect_setup() {
		odo->run();
		left->forward();
		right->backward();
		left->setSpeed(rotate_speed);
		right->setSpeed(rotate_speed);
		current_angle = odo->getAngle();
		in_angle = current_angle;
		wait = false;
		cnt = 0;
		cntinc = 0;
		dec = false;
	}
	void reflect_run() {
		odo->run();
		left->run();
		right->run();
		float lfd = (irlf->getDistance());
		float lbd = (irlb->getDistance());
		if (!wait) {
			if (lfd>10 || lbd>10) {
				cnt = 0;
			}
			else {
				if (lbd>lfd) {
					if(cnt<1) cnt++;
					else dec = true;
					cntinc = 0;
				}
				else {
					if (dec = true) {
						if (cntinc == 0) {
							mid_angle = odo->getAngle();
							out_angle = 2*mid_angle-in_angle;
							cntinc++;
						}
						else {
							wait = true;
						}
					}
					else cnt = 0;
				}
			}
		}

	}
	bool forward_next() {
		bool f = ((irf->getDistance())<reflect_distance);
		bool lf = ((irlf->getDistance())<reflect_distance);
		bool lb = ((irlb->getDistance())<reflect_distance);
		bool r = ((irr->getDistance())<reflect_distance);
		bool b = !uirb->read();
		return (f || lf || lb || r || b);
	}
	bool reflect_next() {
		if (!wait) return false;
		else {
			if ((odo->getAngle())>out_angle()) {
				return true;
			}
		}
		return false;
	}
	void interupt(){
		interuption = true;
	}
	int run(int _channel) {
		channel = _channel;
		if (interuption) {   //interruption handler
			interuption = false;
			return 1;
		}
		else {
			if (channel == 1) {   //move forward
				if (!initialezed) {
					forward_setup();
					initialized = true;
				}
				else {
					forward_run();
					if (forward_next()) {
						channel_stop()
						return 2;
					}
					else return 1;
				}
			}
			if (channel == 2) {   //reflect
				if (!initialized) {
					reflect_setup();
					initialized = true;
				}
				else {
					reflect_run();
					if (reflect_next()) {
						channel_stop();
						return 1;
					}
					else return 2;
				}
			}
		}		
	}
};