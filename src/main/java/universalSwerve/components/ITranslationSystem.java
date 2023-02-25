package universalSwerve.components;

public interface ITranslationSystem {

    void Initialize();

    void SetVelocity(double pVelocity);

    /*
    Returns in inches
    */
    
    double GetDistanceTravelled();
    
}
