import org.apache.commons.math3.analysis.MultivariateMatrixFunction;
import org.apache.commons.math3.analysis.MultivariateVectorFunction;

// Beschreibt, wie eine Funktion so verpackt wird, dass der Optimierer damit etwas anfangen kann.
// nach einem Beipsielcode von Yoshiyuki Arai "TwoDGaussianFunction"


// Wir müssen unsere Funktion in eine Klasse verpacken, damit der Optimierer damit etwas anfangen kann.
// dazu "implementiert" unsere Klasse das "Interface" "MultivariateVectorFunction" aus 

class SimpleRobotFunktion
implements  MultivariateVectorFunction {     // Unsere Klasse implementiert dieses Interface "org.apache.commons.math3.analysis"

  // die folgende Funktion ruft der Optimierer auf, um den Wert der Funktion an einer übergebenen Stelle zu bekommen
  public double[] value(double[] parameter)
  throws IllegalArgumentException { // bedeutet so viel wie "wir behalten uns vor, uns zu beschweren, wenn uns die übergebenen Daten nicht passen.
    double[] ergebnis = new double[3]; // in diesem Array speichern wir unsere Ergebnisse, das aus drei Zahlen (x,y,z Koordinaten) besteht.

    // simpler Beispielroboter: eine Drehbare zentrale Achse, der Endeffektors kann entlang eines Zeigers fahren und sich hoch- und runter bewegen.
    // Die Reihenfolge der Parameter legen wir hier willkürlich fest:
    double winkel=parameter[0];
    double abstand=parameter[1];
    double hoehe=parameter[2];

    // hier berechnen wir die Position des Endeffektors:
    ergebnis[0]=cos((float)winkel)*abstand; //x-Koordinate
    ergebnis[1]=sin((float)winkel)*abstand; //y-Koordinate
    ergebnis[2]=hoehe; //y-Koordinate

      // ... und geben sie an den Optimierer zurück:
    return ergebnis;
  }
}

// jetzt müssen wir dem Optimierer noch eine Möglichkeit geben, die Partiellen Ableitungen der Positionskomponenten nach den Einstellungen zu ermitteln.
// Da wir keine Lust haben, diese Analythisch zu bestimmen, verwenden wir eine Näherung mit finiten Differenzen:

class JacobiAdapter
implements MultivariateMatrixFunction {
  // bei der Konstruktion des JacobiAdapters wird festgelegt, welche Funktion er ableitet
  public JacobiAdapter(MultivariateVectorFunction _baseFunction, int _nParameters, int _nValues) {
    baseFunction=_baseFunction;
    nParameters=_nParameters;
    nValues=_nValues;
  };

  MultivariateVectorFunction baseFunction; // diese Funktion, die abgeleitet wird.
  int nParameters; // wie viele Parameter hat die abgeleitete Funktion?
  int nValues;     // wie viele Dimensionen hat der von ihr ausgegebene Vektor?
  double diffStep=0.01; // Abstand zwischen zwei Funktionsaufrufen im finite-differenzen Schema


  // die folgende Funktion wird aufgerufen, um die Matrize mit den partiellen Ableitungen der Funktion nach allen ihren Parametern zu erhalten:
  public double[][] value(double[] v) {
    double[][] jacobian = new double[nValues][nParameters]; 

    double[]parameterA=new double[nParameters]; // für den ersten Wert einer finiten Differenz
    double[]parameterB=new double[nParameters]; // für den zweiten Wert einer finiten Differenz

    for (int i=0;i<nParameters;i++) {
      // bereite zwei parameter punkte vor und hinter dem gefragten vor, an denen wir die Fuktion aufrufen:
      System.arraycopy( v, 0, parameterA, 0, nParameters );
      System.arraycopy( v, 0, parameterB, 0, nParameters ); 
      parameterA[i]-=diffStep;
      parameterB[i]+=diffStep;
      // rufe die funktion an diesen beiden punkten auf
      double[] valueA=baseFunction.value(parameterA);
      double[] valueB=baseFunction.value(parameterB);
      
      //rechne für jede Komponente des Ergebnisses den zwischen den beiden Funktionsaufrufen  aus..
      for(int o=0;o<nValues;o++){
        jacobian[o][i]=(valueB[o]-valueA[o])/(2*diffStep);
      }
    }
//        println(jacobian[0][0]);
    return jacobian;
  }
}




