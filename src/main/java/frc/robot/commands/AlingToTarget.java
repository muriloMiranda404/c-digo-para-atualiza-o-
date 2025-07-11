package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.SwerveSubsystem;

public class AlingToTarget extends Command {
    
    private double setpointY;
    private double setpointX;

    private final LimelightConfig LimelightConfig;
    
    private Translation2d translation;
    private final SwerveSubsystem subsystem;
    
    private final PIDController xController;
    private final PIDController rotationController;
    
    // Constantes para ajuste fino
    private static final double kP_ROTATION = 0.1;
    private static final double kI_ROTATION = 0.01;
    private static final double kD_ROTATION = 0.005;
    private static final double TOLERANCIA_ROTATION = 0.2;
    
    private static final double kP_X = 0.11;  // Ajuste esses valores conforme necessário
    private static final double kI_X = 0.01;
    private static final double kD_X = 0.005;
    private static final double TOLERANCIA_X = 0.1;
    
    // private static final double TOLERANCIA_ALINHAMENTO = 0.6;
    private static final int MAX_TENTATIVAS = 3;
    private static final double MAX_CORRECAO = 8;
    private static final double ZONA_MORTA = 0.05;
    private static final double MAX_VELOCIDADE_X = 8; 
    private static final double ANGULO_MAXIMO = 30.0;
    private static final double TIMEOUT_SECONDS = 5.5;
    private static final double TEMPO_MINIMO_ESTAVEL = 0.5;
    
    private int tentativas = 0;
    private double ultimaTx = 0.0;
    private double ultimoTempoMudanca = 0.0;
    private boolean ultimoEstadoTarget = false;

    private final Timer timer = new Timer();
    
    public AlingToTarget(LimelightConfig LimelightConfig, SwerveSubsystem subsystem, double setpointX, double setpointY) {
        if (LimelightConfig == null || subsystem == null) {
            throw new IllegalArgumentException("Parâmetros não podem ser nulos");
        }
        this.LimelightConfig = LimelightConfig;
        this.subsystem = subsystem;
        this.setpointX = setpointX;
        this.setpointY = setpointY;
        this.xController = new PIDController(kP_X, kI_X, kD_X);
        this.rotationController = new PIDController(kP_ROTATION, kI_ROTATION, kD_ROTATION);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        try {
            rotationController.reset();
            xController.reset();

            rotationController.setSetpoint(0);
            xController.setSetpoint(0);  

            rotationController.setTolerance(TOLERANCIA_ROTATION);
            xController.setTolerance(TOLERANCIA_X); 
            
            System.out.println("Iniciando alinhamento com a tag...");
            timer.reset();
            timer.start();

            tentativas = 0;
            ultimoEstadoTarget = false;
            ultimaTx = 0.0;
            ultimoTempoMudanca = 0.0;
        } catch (Exception e) {
            System.err.println("Erro na inicialização do alinhamento: " + e.getMessage());
        }
    }
    @Override
    public void execute() {
        try {
            boolean temTarget = LimelightConfig.getHasTarget();
            
            // Detecta perda de alvo
            if (temTarget != ultimoEstadoTarget) {
                ultimoEstadoTarget = temTarget;
                if (!temTarget) {
                    tentativas++;
                    System.out.println("Tag perdida! Tentativa " + tentativas + " de " + MAX_TENTATIVAS);
                }
            }
            
            if (temTarget) {
                double tx = LimelightConfig.getTx();
                double distanciaX = LimelightConfig.getTy();
                double ta = LimelightConfig.getTa();

                if (ta >= 3.5) {
                    ta -= 3.5;
                } else if (ta <= 4) {
                    ta *= 16;
                }
                
                // Proteção contra valores inválidos
                if (Double.isNaN(tx) || Double.isNaN(distanciaX) || Double.isNaN(ta)) {
                    System.err.println("Valores inválidos recebidos da LimelightConfig");
                    return;
                }
                
                // Verifica mudanças bruscas
                if (Math.abs(tx - ultimaTx) > 10.0) {
                    ultimoTempoMudanca = timer.get();
                }
                ultimaTx = tx;
                
                // Só move se estiver estável por tempo suficiente
                if (timer.get() - ultimoTempoMudanca > TEMPO_MINIMO_ESTAVEL) {
                    // Calcula correções
                    double correcaoRotacao = rotationController.calculate(tx, setpointX);
                    double correcaoX = xController.calculate(distanciaX, setpointY);
                    
                    // Aplica zona morta
                    if (Math.abs(correcaoRotacao) < ZONA_MORTA) correcaoRotacao = 0;
                    if (Math.abs(correcaoX) < ZONA_MORTA) correcaoX = 0;

                    // Limita correções
                    correcaoRotacao = Math.min(Math.max(correcaoRotacao, -MAX_CORRECAO), MAX_CORRECAO);
                    correcaoX = Math.min(Math.max(correcaoX, -MAX_VELOCIDADE_X), MAX_VELOCIDADE_X);
                    
                    // Movimento mais suave em ângulos grandes
                    if (Math.abs(tx) > ANGULO_MAXIMO) {
                        correcaoRotacao *= 0.5;
                        correcaoX *= 0.5;
                    }

                    // Limita o valor do TA
                    if (ta > 4) {
                        ta = 4;
                    }

                    correcaoRotacao = -correcaoRotacao;
                    correcaoX = -correcaoX;

                    translation = new Translation2d(correcaoX * ta, 0);
                    subsystem.drive(translation, correcaoRotacao, true);
                    
                    // Debug
                    SmartDashboard.putNumber("Tempo de Alinhamento", timer.get());
                    SmartDashboard.putNumber("Tentativas", tentativas);
                    SmartDashboard.putNumber("Tempo desde última mudança", timer.get() - ultimoTempoMudanca);
                    LimelightConfig.setLedMode(4);
                    
                    System.out.printf("Alinhando - TX: %.2f° | Dist X: %.2f | Rot: %.2f | X: %.2f%n", 
                        tx, distanciaX, correcaoRotacao, correcaoX);
                    SmartDashboard.putNumber("tx", tx);      
                    SmartDashboard.putBoolean("tag achada", LimelightConfig.getHasTarget());
                    SmartDashboard.putNumber("ty", LimelightConfig.getTy());
                    SmartDashboard.putBoolean("foi alinhada", rotationController.atSetpoint());
                    SmartDashboard.putBoolean("foi alinhada", xController.atSetpoint());
                }
            } else {
                subsystem.drive(new Translation2d(), 0, true);
                System.out.println("Nenhuma tag detectada - Verifique o campo de visão da LimelightConfig");
            }
        } catch (Exception e) {
            System.err.println("Erro durante execução do alinhamento: " + e.getMessage());
            subsystem.drive(new Translation2d(), 0, true); // Para o robô com segurança
        }
    }

    @Override
    public boolean isFinished() {
        return (rotationController.atSetpoint() && xController.atSetpoint()) ||
               timer.hasElapsed(TIMEOUT_SECONDS) ||
               tentativas >= MAX_TENTATIVAS;
    }

    @Override
    public void end(boolean interrupted) {
        try {
            subsystem.drive(new Translation2d(), 0, true);
            timer.stop();
            String mensagem;
            if (timer.hasElapsed(TIMEOUT_SECONDS)) {
                mensagem = "Alinhamento interrompido por timeout!";
            } else if (tentativas >= MAX_TENTATIVAS) {
                mensagem = "Alinhamento interrompido após " + tentativas + " tentativas!";
            } else if (interrupted) {
                mensagem = "Alinhamento interrompido pelo usuário!";
            } else {
                mensagem = "Alinhamento concluído com sucesso!";
            }
            System.out.println(mensagem);
        } catch (Exception e) {
            System.err.println("Erro ao finalizar alinhamento: " + e.getMessage());
        }
    }
}