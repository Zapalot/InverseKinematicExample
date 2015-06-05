// Dieses beispielprogramm habe ich von https://github.com/arayoshipta/TwoDGaussainProblemExample abgeleitet.

// in diesen Bibliotheken liegen die Optimierungsfunktionen
import org.apache.commons.math3.*;
import org.apache.commons.math3.fitting.leastsquares.*;


// hier definieren wir die Kinematik unseres Roboters
SimpleRobotFunktion myRobotForward= new SimpleRobotFunktion();
MultivariateMatrixFunction robotJacobian=new JacobiAdapter(myRobotForward, 3, 3);


// Der LevenbergMarquardtOptimizer macht die eigentliche arbeit für uns - bauen wir uns erstmal einen.
// Er soll "singular value decomposition" 8SVD) verwenden, um robust bei singulären Jakobi-Matrizen zu sein.
LevenbergMarquardtOptimizer optimizer= new LevenbergMarquardtOptimizer();

// Damit wir den optimierer anwenden können, müssen wir ihm das Problem in geeigneter Form beschreiben. 
// Dafür ist die LeastSquaresProblem Klasse da, welche wird über einen "LeastSquaresBuilder" erzeugen: 
LeastSquaresBuilder problemBuilder= new LeastSquaresBuilder();

void setup() {
  //Wir wollen unsere vorwärtskinematik als Modell verwenden:
  problemBuilder.model(myRobotForward, robotJacobian);

  //wo soll unser Arm hin?
  double[] targetPoint = {
    0.1, 
    0.2, 
    0.3
  };
  problemBuilder.target(targetPoint);

  //wo fangen wir an?
  double[] startInput = {
    0.5, 
    0.5, 
    0.5
  };
  problemBuilder.start(startInput);

  //wie viele Iterationen lang soll der Optimierer maximal versuchen, ans Ziel zu kommen?
  problemBuilder.maxIterations(100);
  //wie oft soll der Optimierer maximal die Modellfunktion aufgerufen werden (das begrenzt die Zeit die er mir Rechnen verbringt)?
  problemBuilder.maxEvaluations(1000);
}
void draw() {
  try {
    //lass den optimierer rechnen und 
    LeastSquaresOptimizer.Optimum optimum = optimizer.optimize(problemBuilder.build());

    //get optimized parameters
    double[] optimalValues = optimum.getPoint().toArray();
    double[] resultingPoint=myRobotForward.value(optimalValues);
    //output data
    for ( int i=0;i<optimalValues.length;i++) {
      System.out.println("v"+i+": " + optimalValues[i]+"-->"+resultingPoint[i]);
    }
    System.out.println("Iteration number: "+optimum.getIterations());
    System.out.println("Evaluation number: "+optimum.getEvaluations());
  } 
  catch (Exception e) {
    System.out.println(e.toString());
  }
}

