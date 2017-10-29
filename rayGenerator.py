bl_info = {
    "name": "Ray Generator",
    "description": "This addon generates ray animations in the View 3D",
    "author": "Álvaro Martínez Fernández",
    "version": (1, 0),
    "blender": (2, 72, 0),
    "location": "View3D > Misc > Genera Rayos",
    "warning": "", 
    "wiki_url": "",
    "category": "Animation"
}

import bpy
import numpy as np
from mathutils import *

class RayoPanel(bpy.types.Panel):
    """Panel del Addon"""
    bl_label = "Genera Rayos"
    bl_idname = "SCENE_PT_genera_rayos"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_context = "objectmode"

    def draw(self, context):
        layout = self.layout

        scene = context.scene
        
        layout.label(text="Main ray")
        
        r1 = layout.row(align=True)
        r1.prop(scene, "ray_length")
        
        r2 = layout.row(align=True)
        r2.prop(scene, "ray_height")
        
        r3 = layout.row(align=True)
        r3.prop(scene, "ray_max_width")
        
        r4 = layout.row(align=True)
        r4.prop(scene, "ray_resolution")
        
        r5 = layout.row(align=True)
        r5.prop(scene, "ray_frequency")
        
        layout.label(text="Ray Shape")
        
        r6 = layout.row(align=True)
        r6.prop(scene, "ray_radius")
        
        layout.label(text="Distortion")
        
        r7 = layout.row(align=True)
        r7.prop(scene, "ray_max_vel_x")
        r7.prop(scene, "ray_max_vel_y")
        r7.prop(scene, "ray_max_vel_z")
        
        layout.label(text="Subrays")
        
        r8 = layout.row(align=True)
        r8.prop(scene, "num_subRayos")
        
        r9 = layout.row(align=True)
        r9.prop(scene, "length_subRayos")
        
        layout.label(text="Animation")
        r10 = layout.row(align=True)
        r10.prop(scene, "ani_duration")
        
        r11 = layout.row(align=True)
        r11.prop(scene, "multiAnimation")
        
        r12 = layout.row(align=True)
        r12.operator("object.generate_rays", text = "Generate Rays")
        
        r13 = layout.row(align=True)
        r13.operator("object.remove_rays", text = "Remove Rays")
        
        r14 = layout.row(align=True)
        r14.operator("object.reset_values", text = "Reset Values")
#end clase RayoPanel

def createEmpty(typ, loc, nombre):
    """Crea un objeto NUll con un nombre específico y una posición dada"""
    bpy.ops.object.empty_add(type=typ, location=loc)
    bpy.context.object.name = nombre
#endFunction

def borraObjetos(nombre):
    """Borra los objetos que contengan el nombre pasado como parámetro"""
    for o in bpy.data.objects:
        if nombre in o.name:
            bpy.data.scenes[0].objects.unlink(o)
            bpy.data.objects.remove(o)
#endFunction

def searchObject(nombre):
    """
    Busca un objeto que contenga el nombre pasado com argumento y
    devuelve un booleano indicando si lo ha encontrado (True) o si no (False).
    """
    found = False
    for o in bpy.data.objects:
        if nombre in o.name:
            found = True
    return found
#endFunction

def searchMaterial(nombre):
    """Busca un material dado un nombre como argumento"""
    found = False
    for o in bpy.data.materials:
        if nombre in o.name:
            found = True
    return found
#endFunction

def Hermite(t, tiempo, pos, vel):
    """Calcula la interpolación de Hermite"""
    n = np.size(tiempo)
    h = 0.0
    for i in range(1, n):
        if t >= tiempo[i - 1] and t < tiempo[i]:
            u = (t - tiempo[i - 1])/(tiempo[i] - tiempo[i - 1])
            h = (1 - 3*u*u + 2*u*u*u)*pos[i - 1] + u*u*(3 - 2*u)*pos[i] + u*(u - 1)*(u - 1)*vel[i - 1] + u*u*(u - 1)*vel[i]
        elif t >= tiempo[n - 1]:
            h = pos[n - 1]
    return h 
#end function

def get_vector_tg(t, tiempo, pos, vel):
    """Calcula el vector tangente mediante la primera derivada de la función"""
    incr = 0.01
    a = Hermite(t + incr, tiempo, pos, vel)
    b = Hermite(t, tiempo, pos, vel)
    
    return (a - b)/incr
#end function
    
def drawPath(ob, step, recorrido, curva):
    """Convierte la motion path de un objeto en un objeto de tipo curva"""
    scene = bpy.context.scene
    start = scene.frame_start
    end = scene.frame_end
    actual = scene.frame_current
    puntos = list(range(start - 1, end, step))
    
    spline = recorrido.splines.new('BEZIER')
    spline.bezier_points.add(len(puntos) - 1)

    c = 0

    for n in puntos:
        scene.frame_set(n)
        matrix = ob.matrix_world.copy()
        nodo = spline.bezier_points[c]
        nodo.co = matrix.to_translation()
        nodo.handle_right_type='VECTOR'
        nodo.handle_left_type='VECTOR'
        c += 1

    scene.frame_set(actual)
#endFunction

def makeMaterial(name, diffuse, specular, hardness, emit):
    """Crea un material"""
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = diffuse
    mat.diffuse_shader = 'LAMBERT' 
    mat.diffuse_intensity = 1.0 
    mat.specular_color = specular
    mat.specular_shader = 'COOKTORR'
    mat.specular_intensity = 0.5
    mat.specular_hardness = hardness
    mat.ambient = 1
    mat.emit = emit
    return mat
#endFunction

def setMaterial(ob, mat):
    """Asigna un material a un objeto"""
    me = ob.data
    me.materials.append(mat)
#endFunction

def calculateTimes(incr, num_puntos):
    """Calcula los vectores de tiempo para la animación del rayo"""
    tiempos = []
    
    for i in range(0, len(incr)):
        t = []
        for j in range(0, num_puntos):
            t = np.append(t, j*incr[i])
        #end for 
        tiempos.append(t)
    #end for
        
    return tiempos
#endFunction
 
 
class Rayo:
    """
    Clase que se encarga de crear los objetos Rayo y de su animación. Genera los rayos
    en el View 3D mediante curvas y les asigna un material.
    """
    
    """
    Inicializamos las variables que se utilizarán para construir el rayo y asignarle
    un material.
    """
    def __init__(self, length, height, max_width, num_puntos, resolution, max_vel, canon, molde, material):
        self.length =  length
        self.height =  height
        self.max_width =  max_width
        # Deben haber al menos dos puntos para interpolar la curva
        if num_puntos >= 2:
            self.num_puntos =  num_puntos
        else:
            self.num_puntos = 2
        self.resolution =  resolution
        self.max_vel = max_vel
        
        self.pos = np.array(())
        self.vel = np.array(())
        self.tiempo = np.array(())
        
        self.posCanon = canon.location
        self.rotationCanon = canon.rotation_quaternion
        self.matRayo = material
        self.molde = molde
        
        self.incr = resolution/num_puntos
        self.num_frames = int(np.round(resolution*24))
        self.dt = 1.0/24.0
        
        # Creamos el rayo
        self.recorrido = bpy.data.curves.new('recorrido','CURVE')
        self.rayo = bpy.data.objects.new('rayo',self.recorrido)
        self.rayo.rotation_mode = 'QUATERNION'
        bpy.context.scene.objects.link(self.rayo)
        self.recorrido.dimensions = '3D'
        
        # El motionRayo se encarga generar la motion path que representará el rayo
        self.motionRayo = bpy.data.objects.new("motionRayo", None)
        bpy.context.scene.objects.link(self.motionRayo)
        self.motionRayo.rotation_mode = 'QUATERNION'
    #end __init__
    
    """
    Este método se encarga de constuir un rayo en el View 3D. Para ello se hace uso
    del método rando.uniform de numpy con el fin de que el rayo sea más irregular
    y realista.
    Primero se anima el motionRayo interpolando los datos pasados por el interfaz
    con Hermite. A continuación se crea una curva del motion path, se le asigna un
    Bevel Object para que tenga forma tridimensional y por último un material.
    """
    def crearRayo(self):
        # Construimos los vectores de tiempo, velocidades y posiciones para
        # posteriormente interpolarlos con Hermite.
        # La rotación del cañón se tiene en cuenta a la hora de la interpolación.
        for i in np.arange(0, self.num_puntos):
            # El inicio del rayo debe coincidir con la posición del cañón
            if i == 0:
                p = [0, 0, 0]
            else:
                x = np.random.uniform(0, self.max_width)
                y = np.random.uniform((i-1) * self.length / (self.num_puntos - 1), i * self.length / (self.num_puntos - 1))
                
                # El último punto debe coincidir con la dirección del cañón
                if i == self.num_puntos - 1:
                    z = 0
                # Evitamos que el cambio sea muy brusco al principio y al final de la curva
                elif i == 1 or i == self.num_puntos - 2:
                    z = np.random.uniform(0, self.height/3)
                else:
                    z = np.random.uniform(0, self.height)
                
                # Posición del punto. Es de tipo vector para poderlo sumar con
                # otros vectores en la interpolación de Hermite
                p = Vector((x, y, z))
            #end if
            
            v = [self.max_vel]
            
            # Almacenamos los datos en los arrays
            self.pos = np.append(self.pos, p)
            self.vel = np.append(self.vel, v)
            self.tiempo = np.append(self.tiempo, i*self.incr)
        #end for
        
        # Generamos las matrices con los datos para que se puedan interpolar con Hermite
        self.pos = self.pos.reshape(self.num_puntos, 3)
        self.vel = self.vel.reshape(self.num_puntos, 3)
        
        # Ajustamos el frame-end para construir todo el rayo independientemente
        # de la resolución de este
        currentFrameEnd = bpy.context.scene.frame_end
        bpy.context.scene.frame_end = self.num_frames
        
        # En este bucle interpolamos los datos con Hermite y animamos un objeto
        # de tipo Null que luego utilizaremos para crear la forma del rayo
        # Esta interpolación se hace en el origen de coordenadas. Se tiene en cuenta
        # la orientación del cañón, pero no su posición. Posteriormente se desplaza
        # el rayo a la localización del cañón. Esto se hace para que no se deforme
        # la interpolación.
        for f in range(1, self.num_frames):
            # El tiempo en segundos
            t = f*self.dt
            
            # En el primer fotograma la curva parte del origen de coordenadas
            if f == 1:
                p = [0, 0, 0]
            else:
                # Interpolamos los puntos con Hermite
                p = Vector((Hermite(t, self.tiempo, self.pos, self.vel)))
                # Rotamos el punto utilizando la rotación del cañón
                p.rotate(self.rotationCanon)
            
            # Asignamos la posición al objeto motionRayo que trazará la trayectoria
            self.motionRayo.location = p
            # Guardamos la posición en el Timeline
            self.motionRayo.keyframe_insert(data_path="location", frame=f)
        #end for
        
        # Creamos una curva de Bezier a partir del movimiento interpolado
        drawPath(self.motionRayo, 1, self.recorrido, self.rayo)
        
        # Volvemos al frame_end inicial
        bpy.context.scene.frame_end = currentFrameEnd
        
        # Movemos el rayo hasta la posición del cañón
        self.rayo.location += Vector((self.posCanon))
        
        # Aplicamos un molde a la curva para darle volumen
        self.rayo.data.bevel_object = self.molde
        # Cerramos los extremos de la curva
        self.rayo.data.use_fill_caps = True
        
        # Asignamos el material al rayo
        setMaterial(self.rayo, self.matRayo)
    #end crearRayo
    
    """Devuelve la referencia de la curva que representa al rayo"""
    def getRayo(self):
        return self.rayo
    #end
    
    """Anima un subrayo con respecto a la curva del objeto"""
    def animarSubrayo(self, subRayo, angulo, duration, tiempo):
        # Animamos el rayo a partir de la posición actual del Timeline
        fcurrent = bpy.context.scene.frame_current
        totalFrames = int(np.round(duration*24)) + fcurrent
        for f in range(fcurrent, totalFrames):
            t = (f - fcurrent + 1)*self.dt
            s = [0.0, 0.0, 0.0]
            
            # Rotamos el subrayo con respecto a la tangente de la curva
            # mediante cuaterniones
            vt = Vector((get_vector_tg(t, tiempo, self.pos, self.vel)))
            vt.normalize()
            vx = Vector((1, 0, 0))
            vq = vx.cross(vt)
            vq.normalize()
            aq =  np.arccos(vt.dot(vx)) + angulo
            q = Quaternion(vq, aq)
            
            # En el primer fotograma empieza en el origen de coordenadas
            if f == fcurrent:
                p = self.pos[0]
            else:
                # Interpolamos el subrayo con su tiempo correspondiente
                p = Vector((Hermite(t, tiempo, self.pos, self.vel)))
                # Lo rotamos con respecto a la orientación del cañón
                p.rotate(self.rotationCanon)
                # Lo desplazamos hasta la posición del cañón
                p += Vector((self.posCanon))
                # Lo escalamos para que aparezca al principio y desaparezca al
                # final de la animación
                if f == fcurrent or f == totalFrames - 2:
                    s = [0.5, 0.5, 0.5]
                elif f == totalFrames - 1:
                    s = [0.0, 0.0, 0.0]
                else:
                    s = [1.0, 1.0, 1.0]
            
            # Guardamos los fotogramas en el Timeline
            subRayo.location = p
            subRayo.scale = s
            subRayo.rotation_quaternion = q
            subRayo.keyframe_insert(data_path="location", frame=f)
            subRayo.keyframe_insert(data_path="scale", frame=f)
            subRayo.keyframe_insert(data_path="rotation_quaternion", frame=f)
        #end for
    #end animarSubrayo
    
    """Anima el rayo para que aparezca y desaparezca"""
    def animarRayo(self, duration):
        fcurrent = bpy.context.scene.frame_current
        totalFrames = int(np.round(duration*24)) + fcurrent
        
        # Nos aseguramos de que el rayo no se vea antes del comienzo de la animación.
        # Para ello lo escalamos al (0, 0, 0).
        self.rayo.scale = [0.0, 0.0, 0.0]
        self.rayo.keyframe_insert(data_path="scale", frame=1)
        self.rayo.keyframe_insert(data_path="scale", frame=fcurrent - 1)
        
        for f in range(fcurrent, totalFrames):
            t = (f - fcurrent + 1)*self.dt
            s = [0.0, 0.0, 0.0]
            
            # Lo escalamos para que aparezca al principio y desaparezca al
            # final de la animación
            if f == fcurrent or f == totalFrames - 2:
                s = [0.5, 0.5, 0.5]
            elif f == totalFrames - 1:
                s = [0.0, 0.0, 0.0]
            else:
                s = [1.0, 1.0, 1.0]
            
            # Guardamos los fotogramas en el Timeline
            self.rayo.scale = s
            self.rayo.keyframe_insert(data_path="scale", frame=f)
        #end for
    #end animarRayo
#end clase Rayo

# Clase que genera los rayos
class GeneraRayos(bpy.types.Operator):
    """Generador de Rayos"""
    bl_idname = "object.generate_rays"
    bl_label = "Generador de Rayos"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        length = context.scene.ray_length               # Longitud del rayo
        height = context.scene.ray_height               # Altura del rayo
        max_width = context.scene.ray_max_width         # Máximo ancho del rayo
        num_puntos = context.scene.ray_frequency        # Frecuencia del rayo, equivalente al número de puntos
        resolution = context.scene.ray_resolution       # Resolución del rayo; equivale al tiempo en Hermite
        radio = context.scene.ray_radius                # Grosor del rayo
        max_vel_x = context.scene.ray_max_vel_x         # Velocidad en el eje X a la hora de interpolar el rayo con Hermite
        max_vel_y = context.scene.ray_max_vel_y         # Velocidad en el eje y a la hora de interpolar el rayo con Hermite
        max_vel_z = context.scene.ray_max_vel_z         # Velocidad en el eje z a la hora de interpolar el rayo con Hermite
        number_subrayos = context.scene.num_subRayos    # Número de suvrayos que se animarán
        length_subrayos = context.scene.length_subRayos # Longitud de los subrayos
        duration = context.scene.ani_duration           # Duración de la animación
        multiAnimation = context.scene.multiAnimation   # Múltiples rayos
        
        # Vector que contiene los vectores de tiempos del rayo y de los subrayos
        tiempos = []
        
        # Los subrayos deben tener un tamaño igual o menor al del rayo principal
        if length_subrayos > length:
            length_subrayos = length
            context.scene.length_subRayos = length
        
        # Calculamos el número de frames necesarios para la animación
        num_frames = int(np.round(resolution*24))
        
        # Incremento del tiempo
        dt = 1.0/24.0
        
        # Escena actual
        scn = bpy.context.scene
        
        # Creamos el cañón por donde saldrá el rayo. Si no existe se crea uno
        if not searchObject('canon'):
            canon = bpy.data.objects.new("canon", None)
            scn.objects.link(canon)
            canon.empty_draw_type = 'CIRCLE'
            canon.location = (0, 0, 0)
            canon.scale = [0.2, 0.2, 0.2]
            canon.rotation_mode = 'QUATERNION'
        else:
            canon = bpy.data.objects['canon']
        
        # Creamos el molde del rayo. Si no existe se crea uno con forma circular
        if not searchObject('molde'):
            bpy.ops.curve.primitive_bezier_circle_add(location=(0, 0, 0))
            bpy.context.object.name = "molde"
        
        # Actualizamos el radio del rayo  
        molde = bpy.data.objects['molde']
        molde.scale = (radio, radio, radio)
        
        # Creamos el material si no existe. Por defecto es un material de color
        # azul que emite luz. Este se puede editar con Blender una vez creado.
        if not searchMaterial('materialRayo'):
            matRayo = makeMaterial('materialRayo', (0.00520488, 0.203044, 0.514918), (1,1,1), 5, 3)
        else:
            matRayo = bpy.data.materials['materialRayo']
        
        # Si está activado el checkbox de Several Rays no se borran los rayos anteriores
        if multiAnimation == False:
            borraObjetos('rayo')
        
        # Creamos los tiempos para cada rayo
        # Array con los incrementos de tiempo para cada rayo/subrayo
        array_incr = []
        # Array con el número de puntos para cada rayo/subrayo
        array_npuntos = []
        # Array con las duraciones de los rayos/subrayos
        array_duration = []
        
        # Este factor es la relación entre la longitud del rayo principal y la longitud
        # de los subrayos. Sirve para calcular posteriormente el número de puntos para
        # cada subrayo y su resolución
        factor = length/length_subrayos
        
        # El primer elemento del array de incrementos corresponde al rayo principal
        array_incr.append(resolution/num_puntos)
        
        # Se calculan los incrementos y numero de puntos para cada rayo/subrayo
        for i in range(1, number_subrayos + 1):
            array_incr.append(duration/number_subrayos/num_puntos*i)
            array_duration.append(duration/number_subrayos*i)
        
        # Generamos el vector con los vectores de tiempos
        tiempos = calculateTimes(array_incr, num_puntos)
        
        # Creamos los rayos
        # Esta variable sirve para modificar la rotación de los subrayos alrededor del rayo principal
        angulo = 360.0/number_subrayos
        
        # Creamos el rayo principal
        rayoPrincipal = Rayo(length, height, max_width, num_puntos, resolution, [max_vel_x, max_vel_y, max_vel_z], canon, molde, matRayo)
        rayoPrincipal.crearRayo()
        
        # Creamos los subrayos
        for i in range(0, number_subrayos):
            a = Rayo(length_subrayos, height, max_width, np.ceil(num_puntos/factor), np.ceil(resolution/factor), [max_vel_x, max_vel_y, max_vel_z], canon, molde, matRayo)
            a.crearRayo()
            # Animamos cada subrayo creado con el rayo principal
            rayoPrincipal.animarSubrayo(a.getRayo(), i*angulo, array_duration[i], tiempos[i + 1])
        #end for
        
        # Animamos el rayo principal
        rayoPrincipal.animarRayo(duration)
        
        # Eliminamos los motionRayo creados
        borraObjetos('motionRayo')

        return {'FINISHED'}
    #end def execute
#end clase PitolaRayos

class RemoveRays(bpy.types.Operator):
    """Elimina todos los rayos de la escena"""
    bl_idname = "object.remove_rays"
    bl_label = "Elimina todos los rayos"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        borraObjetos('rayo')
        
        return {'FINISHED'}
    #end def execute
#end clase ResetValues

class ResetValues(bpy.types.Operator):
    """Resetea los Valores"""
    bl_idname = "object.reset_values"
    bl_label = "Resetea los Valores"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        # Escena actual
        scn = context.scene
        
        scn.ray_length = 15.0
        scn.ray_height = 2.0
        scn.ray_max_width = 0.5
        scn.ray_resolution = 2.0
        scn.ray_frequency = 10
        scn.ray_radius = 0.03
        scn.ray_max_vel_x = 2.0
        scn.ray_max_vel_y = 2.0
        scn.ray_max_vel_z = 2.0
        scn.num_subRayos = 3
        scn.length_subRayos = 2.0
        scn.ani_duration = 1.0
        
        return {'FINISHED'}
    #end def execute
#end clase ResetValues

"""Registramos las propidades del panel"""
def register():
    # Longitud
    bpy.types.Scene.ray_length = bpy.props.FloatProperty(
        name="length",
        description="Ray length",
        default = 15.0,
        min = 1.0,
        max = 200.0,
        step = 10.0)
    
    # Altura
    bpy.types.Scene.ray_height = bpy.props.FloatProperty(
        name="height",
        description="Ray height",
        default = 2.0,
        min = 0.5,
        max = 30.0,
        step = 10.0)
        
    # Max Width
    bpy.types.Scene.ray_max_width = bpy.props.FloatProperty(
        name="max width",
        description="Maximum ray width",
        default = 0.5,
        min = 0.01,
        max = 10.0,
        step = 10.0)
    
    # Resolución
    bpy.types.Scene.ray_resolution = bpy.props.FloatProperty(
        name="resolution",
        description="Ray resolution",
        default = 2.0,
        min = 0.1,
        max = 10.0,
        step = 10.0)
    
    # Frecuencia
    bpy.types.Scene.ray_frequency = bpy.props.IntProperty(
        name="ray frequency",
        description="Ray frequency",
        default = 10,
        min = 2,
        max = 100,
        step = 1)
        
    # Radio
    bpy.types.Scene.ray_radius = bpy.props.FloatProperty(
        name="radius",
        description="Ray radius",
        default = 0.03,
        min = 0.001,
        max = 2.0,
        step = 1)
        
    # Distorsión en el eje X
    bpy.types.Scene.ray_max_vel_x = bpy.props.FloatProperty(
        name="X",
        description="Maximum X axis distortion",
        default = 2.0,
        min = 0.0,
        max = 6.0,
        step = 10.0)
    
    # Distorsión en el eje Y
    bpy.types.Scene.ray_max_vel_y = bpy.props.FloatProperty(
        name="Y",
        description="Maximum Y axis distortion",
        default = 2.0,
        min = 0.0,
        max = 10.0,
        step = 10.0)
    
    # Distorsión en el eje Z
    bpy.types.Scene.ray_max_vel_z = bpy.props.FloatProperty(
        name="Z",
        description="Maximum Z axis distortion",
        default = 2.0,
        min = 0.0,
        max = 10.0,
        step = 10.0)
    
    # Número de subrayos
    bpy.types.Scene.num_subRayos = bpy.props.IntProperty(
        name="number",
        description="Number of subrays",
        default = 3,
        min = 1,
        max = 20,
        step = 1)
    
    # Longitud de los subrayos
    bpy.types.Scene.length_subRayos = bpy.props.FloatProperty(
        name="length",
        description="Subray length",
        default = 2.0,
        min = 1.0,
        max = 50.0,
        step = 10.0)
    
    # Duración de la animación 
    bpy.types.Scene.ani_duration = bpy.props.FloatProperty(
        name="duration",
        description="Animation duration",
        default = 1.0,
        min = 0.01,
        max = 5.0,
        step = 10.0)
    
    # Múltiples animaciones de rayos
    bpy.types.Scene.multiAnimation = bpy.props.BoolProperty(
        name="Several Rays",
        description="Creates multiple rays",
        default = False)
    
    bpy.utils.register_class(RayoPanel)
    bpy.utils.register_class(GeneraRayos)
    bpy.utils.register_class(RemoveRays)
    bpy.utils.register_class(ResetValues)
#end register


def unregister():
    bpy.utils.unregister_class(RayoPanel)
    bpy.utils.unregister_class(GeneraRayos)
    bpy.utils.unregister_class(RemoveRays)
    bpy.utils.unregister_class(ResetValues)
#end unregister


# This allows you to run the script directly from blenders text editor
# to test the addon without having to install it.
if __name__ == "__main__":
    register()
