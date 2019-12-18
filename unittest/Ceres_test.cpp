#include<iostream>
#include<ceres/ceres.h>
#include<math.h>
using namespace std;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
#define pi 3.141593


class AnalyticCostFunctor : public ceres::SizedCostFunction<1,4> {
   public:
     AnalyticCostFunctor(const double x, const double y) : data_x(x), data_y(y) {}
     virtual ~AnalyticCostFunctor() {}
     virtual bool Evaluate(double const* const* parameters,
                           double* residuals,
                           double** jacobians) const {
            const double a = parameters[0][0];
            const double b = parameters[0][1];
            const double c = parameters[0][2];
            const double d = parameters[0][3];
            const double theta = atan2((data_y-d)*a, (data_x-c)*b);
            double ro;
            if(cos(theta*2) > 0)
              ro = sqrt(cos(theta*2));
            else
              {
                ro = 5;
              }
            // cout << "ro: " << ro << endl;  
            // 残差矩阵
            residuals[0] = pow(data_y-b*ro*sin(theta)-d,2) + pow(data_x-a*ro*cos(theta)-c,2);
            if (!jacobians) return true;
            double*jacobian = jacobians[0];
            if (!jacobian) return true;
            // 雅可比矩阵
            jacobian[1] = -ro*sin(theta)*(data_y - b*ro*sin(theta)-d)*2;
            jacobian[0] = -ro*cos(theta)*(data_x - a*ro*cos(theta)-c)*2;
            jacobian[2] = -2*(data_x - a*ro*cos(theta)-c);
            jacobian[3] = -2*(data_y - b*ro*sin(theta)-d);
            return true;
     }

   private:
     const double data_x;
     const double data_y;
 };



//example指数函数损失函数
struct ExponentialResidual {
  ExponentialResidual(double x, double y)
      : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T* const m, const T* const c, T* residual) const {
    residual[0] = T(y_) - exp(m[0] * T(x_) + c[0]);
    return true;
  }

 private:
  // Observations for a sample.
  const double x_;
  const double y_;
};



int main(int argc, char** argv) {
    // google::InitGoogleLogging(argv[0]);

    //example 指数函数拟合
  
    int kNumObservations = 121;//有50对点待拟合
    
    double data[242]={
      46.339439,23.584583,
      46.482948,23.445282,
      46.626457,23.305981,
      46.769966,23.166679,
      47.918037,22.052269,
      48.061546,21.912968,
      48.205055,21.773666,
      48.779091,21.216461,
      48.921627,21.076172,
      49.063190,20.934895,
      49.629444,20.369785,
      49.771008,20.228508,
      49.912571,20.087231,
      50.054768,19.946590,
      50.197590,19.806585,
      50.483234,19.526577,
      50.626057,19.386574,
      50.911701,19.106565,
      51.054523,18.966562,
      51.340168,18.686554,
      51.625813,18.406548,
      52.054279,17.986534,
      52.197102,17.846531,
      52.482746,17.566523,
      52.768391,17.286516,
      52.911213,17.146511,
      53.196857,16.866505,
      53.339680,16.726500,
      53.625324,16.446493,
      53.768147,16.306488,
      54.203434,15.893669,
      54.350803,15.758459,
      54.792912,15.352831,
      54.940281,15.217623,
      55.087650,15.082415,
      55.529766,14.676790,
      55.677139,14.541583,
      55.978081,14.278131,
      56.735596,13.625302,
      56.887096,13.494737,
      57.192032,13.235916,
      57.658127,12.858088,
      57.813492,12.732145,
      57.968857,12.606202,
      58.124222,12.480259,
      58.756691,11.990479,
      58.915726,11.869198,
      59.074760,11.747917,
      59.557056,11.391096,
      59.719551,11.274496,
      59.882046,11.157896,
      60.044540,11.041296,
      60.537102,10.698792,
      60.702980,10.587055,
      61.707932,9.931479,
      62.046143,9.717902,
      62.216900,9.613823,
      62.561718,9.411089,
      62.734127,9.309722,
      62.906536,9.208354,
      63.255932,9.013683,
      63.431393,8.917690,
      63.782310,8.725703,
      63.960510,8.634896,
      64.316895,8.453282,
      65.399574,7.936019,
      65.582848,7.855935,
      65.766121,7.775849,
      65.949387,7.695764,
      66.133804,7.618434,
      66.504959,7.469289,
      66.690536,7.394717,
      66.877220,7.323034,
      67.440590,7.116652,
      67.630257,7.053175,
      68.200111,6.865493,
      68.391518,6.807517,
      68.582924,6.749540,
      68.774330,6.691566,
      69.353493,6.534750,
      69.547295,6.485438,
      69.839119,6.415914,
      70.033669,6.369566,
      70.424828,6.286094,
      70.620750,6.245894,
      71.408081,6.104365,
      72.002075,6.019689,
      72.200806,5.997249,
      72.399536,5.974809,
      72.598267,5.952370,
      72.797577,5.935824,
      72.996887,5.919278,
      73.595642,5.881691,
      73.795364,5.871171,
      73.995316,5.866810,
      74.595238,5.861746,
      74.795235,5.862730,
      74.995171,5.866836,
      75.594620,5.891780,
      75.794144,5.905521,
      76.192734,5.938543,
      76.391800,5.957824,
      76.789017,6.004739,
      76.987473,6.029589,
      77.382965,6.089249,
      77.973480,6.195144,
      78.169403,6.235200,
      78.364746,6.278112,
      78.560089,6.321025,
      78.948074,6.418325,
      79.141296,6.469858,
      79.333755,6.524272,
      79.526215,6.578688,
      79.907654,6.699088,
      80.286209,6.828183,
      80.848328,7.037787,
      81.033821,7.112494,
      81.218147,7.190105,
      81.401436,7.270103,
      81.945526,7.522867,
      82.125130,7.610859
    };


    double a = 40;
    double b = 40;
    double c = 40;
    double d = 30;
   
    double theta;
    double ro;
    //构造8字形函数
    // for(int i = 0; i < kNumObservations; i++)
    // {
    //     theta = -pi/4 + 0.01*i + 0.01;
    //     ro = sqrt(cos(2*theta));
    //     data[2*i] = a*ro*cos(theta) + c;
    //     data[2*i+1] = b*ro*sin(theta) + d;
    //     cout<<data[2*i]<<"\t"<<data[2*i+1]<<endl;
    // }

    // The variable to solve for with its initial value.
    double a_init = 20;
    double b_init = 20;
    double c_init = 50;
    double d_init = 20;
    double param[4] = {a_init,b_init,c_init,d_init};
    cout << data[1] << "   " << data[2] << endl;
    cout << "go_____: " << data[1]-b*ro*sin(theta) + data[0]-a*ro*cos(theta) << endl;
    // Build the problem.
    Problem problem;
    for (int i = 0; i < kNumObservations; i++) {
    CostFunction *cost_function = new AnalyticCostFunctor(data[2*i], data[2*i+1]);
    problem.AddResidualBlock(cost_function, NULL, param);
    }


  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).

//   // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "a : " << a_init
            << " -> " << param[0] << "\n";
  std::cout << "b : " << b_init
            << " -> " << param[1] << "\n";
 std::cout << "c : " << c_init
            << " -> " << param[2] << "\n";
 std::cout << "d : " << b_init
            << " -> " << param[3] << "\n";
  return 0;




}
