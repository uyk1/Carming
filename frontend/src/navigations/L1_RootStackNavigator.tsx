import {createNativeStackNavigator} from '@react-navigation/native-stack';
import {L2_LandingStackNavigator} from '.';
import L2_AppDrawerNavigator from './L2_AppDrawerNavigator';
import {useEffect} from 'react';
import SplashScreen from 'react-native-splash-screen';
import LaunchScreen from './../screens/LaunchScreen';

const Stack = createNativeStackNavigator();

function RootStackNavigator() {
  useEffect(() => {
    SplashScreen.hide();
  }, []);

  return (
    <Stack.Navigator
      initialRouteName="LaunchScreen"
      screenOptions={{
        headerShown: false,
      }}>
      <Stack.Screen name="LaunchScreen" component={LaunchScreen} />
      <Stack.Screen name="LandingStack" component={L2_LandingStackNavigator} />
      <Stack.Screen name="AppDrawer" component={L2_AppDrawerNavigator} />
    </Stack.Navigator>
  );
}

export default RootStackNavigator;
