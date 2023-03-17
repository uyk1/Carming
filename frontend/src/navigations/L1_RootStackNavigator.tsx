import {createNativeStackNavigator} from '@react-navigation/native-stack';
import {L2_LandingStackNavigator} from '.';
import L2_AppDrawerNavigator from './L2_AppDrawerNavigator';

const Stack = createNativeStackNavigator();

function RootStackNavigator() {
  return (
    <Stack.Navigator
      initialRouteName="LandingStack"
      screenOptions={{
        headerShown: false,
      }}>
      <Stack.Screen name="LandingStack" component={L2_LandingStackNavigator} />
      <Stack.Screen name="AppDrawer" component={L2_AppDrawerNavigator} />
    </Stack.Navigator>
  );
}

export default RootStackNavigator;
