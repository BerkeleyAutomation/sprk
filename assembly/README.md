This outlines step-by-step instruction on building and dassembing the SPR2.

## Base assembly construction
# Fabrication of the base plate
The base plate is fabricated by laser cutting a 1/8th inch clear acrylic sheet using the file here (add link). Additional holes can be made onto this base plate to fixture it to the working table. We are going to attach the servos directly onto this base plate, and need to add a countersink to the servo attach holes in order to use the Robotis Rivets. These counter sinks on the underside of the base plate can be made using any larger diameter drill bit on a drill press. The depth of the countersink can be up to 60 thou of an inch to firmly fix the servo onto the plate. The exact depth can be found by gradually increasing countersink depth till the rivets reach the servo. The countersinks also help to keep the rivets on the bottom flush with the surface of the base plate.

# Fabrication of the servo horns
The servo horns are fabricated using 1/4th inch Delrin sheets. Each horn has 4 holes to fix them to the servos using the Robotis rivets, and 1 hole to fix one end of the M3 nylon ball joint rods. Once these horns are laser cut using the files here (add link), they need to be post processed on a drill press. The rivets are not very long, so a countersink also needs to be made to allow the rivets to reach the servo. Note that the default servo horn was not removed and the Delrin horn was simply attached to it using the rivets. The exact counter sink depth on the Delrin horn can be found similarly to that with the base plate. Approximately, this depth can be up to 0.15 inch. The fifth hole – for the Nylon ball joint rod – can be drilled for an M3 screw thread. We use Delrin for the servo horn to be able to thread this hole. Repeat this process for 6 servo horns.

# Fixing horns to servos and servos to base plate using rivets
Now that the base plate and servo horns are fabricated, we can put together the base assembly. First, we need to home all the servos, and attach the servo horns such that the hole for the ball joint rod is pointing parallel to the body of the servo (as shown in the image). Each of the horns can now be attached to the servos using the small rivets. And subsequently, we can attach the 6 servos to the base plate. It is better to do testing of the code before attaching the servos to the base plate, because the servo horns can collide with the base plate.

## Top assembly construction
# Fabrication of the top plate and buttress
The top plate is also laser cut from a 1/8th inch acrylic sheet using the file here (add link). The plate has three holes for attaching mounts for different subtasks. More holes can be added for more complex mounts. The Delrin buttress will help attach the Nylon ball joint rod to the top plate. So, we need to drill the two holes on each buttress for an M3 thread after laser cutting. Once this is completed for all 3 buttresses, we can press fit each of them into the top plate.

## Putting together base and top assemblies
When performing any kind of the assembly with the servos, it is always good to disconnect power to the servos in order to prevent any damage to them. The servos can also be homed beforehand so we don’t have to rotate the horn by hand.
Now we can go ahead and thread the other end of the Nylon ball joint rods to the Delrin servo horns. Hold the servo horn stationary with one hand, when threading the rods into the horns. This is the last step, and the completed platform assembly should look as follows.

## Some things to take care
*Assigning servo IDs
Dynamixel XL-320s can be assigned Servo IDs to operate them individually when they are daisy-chained. Assigning servo IDs should be the first thing you should do before operating a new servo, because sometimes XL-320s are not able to change IDs afterwards and require a firmware reset. Furthermore, assigning unique and sequential IDs to the six servos can help simplify the programming a lot.

*Assigning servo limits
There can be a lot of collisions when testing different motion modes. Majority of these collisions involve the servo horn hitting the base plate. To avoid having to replace parts later on due to damage, assign hard limits on the servo position in code such that it can never be commanded to cause a collision.

*Debugging servo problems
If a servo overheats or stops working for some reason (usually indicated with a warning light on the servo), you can try turning off the power, waiting for some time, and then restarting the system. Resetting the torque limit and joint mode for each servo can also help.
However, if a replacement of a servo is required, note the servo ID of the faulty servo (so you can initialize the ID of the replacement servo accordingly), unthread the screw for the the ball joint rod from that servo, and remove the 4 rivets on the bottom of that servo. After all wires are disconnected, you can swap in the new replacement servo.
