class State {
	Location* location;
	Location* target;
	Odometry* odo;
	Gyroscope* gyr;
	Motor* left;
	Motor* right;
	IR* ir1,ir2,ir3,ir4;
	bool map [300][300];
public:
	State(int ldirp, int rdirp, int lpwmp, int rpwmp,int rhall, int lhall, int IR1p, int IR2p, int IR3p, int IR4p, bool* setmap)
	{
		left = new Motor(ldirp,lpwmp,lhall,0);
		right = new Motor(rdirp,rpwmp,rhall,1);
		ir1 = new IR(IR1p);
		ir2 = new IR(IR2p);
		ir3 = new IR(IR3p);
		ir4 = new IR(IR4p);
		setmap = map;;
	}

	void reset (Location _location) {
		odo->set(_location);
		gyr->set(_location->theta());
	}

	void setTarget (Location _target) {
		target = _target;
	}

	void track () {

	}

	bool planwork () {

	}


	Location[] plan() {

	}

	void approach () {
		Location[] Myplan = plan();
		while ()


	}
};
