// helper functions, to keep example code "clean"

// trim double-precision number to specified precision
double trimPlaces(double v, int p) { return floor(v*pow(10.0,p))/pow(10.0,p); }

// convert double-precision number to string, using specified precision
std::string toString(double v, int p) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(p) << trimPlaces(v,p-1);
  std::string result = ss.str();

  return result.substr(0, result.size()-1);
}

// update Pi-estimate with next iter step
double updatePi(double& calcPi)
{
  static int sign, denom, iter;

  if (calcPi==0) { calcPi=0; sign=1; denom=1; }

  // use Leibniz method
  double inc = 4.0 / denom;
  calcPi += sign * inc;
  denom += 2;
  sign = -sign;

  return fabs(calcPi-M_PI);
}

