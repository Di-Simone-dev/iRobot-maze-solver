import py_trees
from helpers import *

#Secondo subtree del Pledge
#controlla le rotazioni e se non è necessario farne ritorna successful per per permettere il movimento
class FollowWall(py_trees.behaviour.Behaviour):
    def __init__(self, name="Follow Wall"):
        super().__init__(name)

    #se implemento qui il aligned with global non serve più altrove
    def update(self):
        pose = BB.get("pose")
        heading = BB.get("heading")
        forward = forward_cell(pose, heading)
        
        global_heading = BB.get("heading_global")
        if BB.get("pledge_counter") == 0 and BB.get("heading") == BB.get("heading_global") and is_free(forward):
            #print("HEADING CORRETTO", BB.get("pledge_counter"))
            return py_trees.common.Status.SUCCESS
        else:
            #pose = BB.get("pose")
            #heading = BB.get("heading")
            #global_heading = BB.get("heading_global")
            counter = BB.get("pledge_counter")

            visited = BB.get("visited")
            #forward = forward_cell(pose, heading)
            left = left_cell(pose, heading)
            right = right_cell(pose, heading)
            backright = backright_cell(pose,heading)
            backleft = backleft_cell(pose,heading)
            back = backward_cell(pose,heading)

            if(is_free(left) and is_free(right) and is_free(back) and "Turn" not in BB.get("last_action")):
                print("Scelta tra sx e dx con il seguente pledge counter", counter)
                print(left, right)
            if(is_free(left) and is_free(forward) and is_free(back) and "Turn" not in BB.get("last_action")):
                print("Scelta tra sx e dritto con il seguente pledge counter", counter)
                print(left,forward)
            if(is_free(right) and is_free(forward)and is_free(back) and "Turn" not in BB.get("last_action")):
                print("Scelta tra dx e dritto con il seguente pledge counter", counter)
                print(right, forward)
            #implemento il controllo sul pledge counter 
            #>0 prioritizzo il turn left, mentre <0 prioritizzo il turn right
            change_heading = True
            if BB.get("pledge_counter")>0:
                #print("priorittizo la sinistra")
                #controlli aggiuntivi necessari, se ha appena girato a sinistra senza muoversi non deve rifarlo
                if is_free(left) and not is_free(backleft) and "Turn Left" not in BB.get("last_action"):
                    BB.set("heading", (heading - 90) % 360)
                    BB.set("last_action", "Turn Left (wall follow)")
                    #print("LEFT111 seguo il muro a sinistra" )
                #ALTRIMENTI DESTRA E POI CENTRO
                elif is_free(forward):
                    BB.set("heading", heading)
                    BB.set("last_action", "Continue forward")
                    change_heading = False
                    #print("VADO DRITTO")
                elif is_free(right) and not is_free(backright):
                    BB.set("heading", (heading + 90) % 360)
                    BB.set("last_action", "Turn Right (wall follow)")
                    #print("RIGHT111 seguo il muro a destra" )

                elif is_free(back):# QUI DEVO CAPIRE come gestire il pledgecounter ipotizzo di girare a sx 2 volte
                    #POSSO FARLO ANCHE IN 2 STEP (PROBABILMENTE PIù ADATTO)
                    BB.set("heading", (heading - 180) % 360)
                    BB.set("last_action", "Turn Back (2 times right)")
                    #print("TORNO INDIETRO")
                else:
                    BB.set("last_action", "Wall ended → no free path")
                    #print("FAIL")
                    return py_trees.common.Status.FAILURE


            #Qui invece prioritizzo la destra        
            elif BB.get("pledge_counter")<=0:
                #print("priorittizo la destra")
                #LE SVOLTE VANNO FATTE 1 SOLA VOLTA DI FILA 
                if is_free(right) and not is_free(backright) and "Turn Right" not in BB.get("last_action"):
                    BB.set("heading", (heading + 90) % 360)
                    BB.set("last_action", "Turn Right (wall follow)")
                    #print("RIGHT111 seguo il muro a destra" )
                elif is_free(forward):
                    BB.set("heading", heading)
                    BB.set("last_action", "Continue forward")
                    change_heading = False
                    #print("VADO DRITTO")
                elif is_free(left) and not is_free(backleft):
                    BB.set("heading", (heading - 90) % 360)
                    BB.set("last_action", "Turn Left (wall follow)")
                    #print("LEFT111 seguo il muro a sinistra" )

                elif is_free(back):# non dovrebbe servire questo controllo
                    BB.set("heading", (heading - 180) % 360)
                    BB.set("last_action", "Turn Back (2 times left)")
                    #print("TORNO INDIETRO")
                else:
                    BB.set("last_action", "Wall ended → no free path")
                    #print("FAIL")
                    return py_trees.common.Status.FAILURE

            if(change_heading == False):
                return py_trees.common.Status.SUCCESS   #se non giro mi muovo procedendo con il moveforward
            else:
                return py_trees.common.Status.FAILURE   #altrimenti resto fermo