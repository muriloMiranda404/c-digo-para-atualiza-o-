package frc.robot.constants;

public enum Port {
    //exemple
    
    TESTE("nada", 7),
    MESMO("intaek", 8),
    NADA("elevador", 4);


    public String name;
    public int id;
    private Port(String name, int ID){
        this.name = name;
        this.id = ID;
    }
}
