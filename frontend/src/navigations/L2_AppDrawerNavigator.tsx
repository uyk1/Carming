import {createDrawerNavigator} from '@react-navigation/drawer';
import L3_MainDrawerNavigator from './L3_MainDrawerNavigator';
import L3_TotalJourneyStackNavigator from './L3_TotalJourneyStackNavigator';

const Drawer = createDrawerNavigator();

function L2_AppDrawerNavigator() {
  return (
    <Drawer.Navigator
      initialRouteName="Main"
      screenOptions={{
        drawerPosition: 'right',
        headerShown: false,
      }}>
      <Drawer.Screen name="Main" component={L3_MainDrawerNavigator} />
      <Drawer.Screen
        name="TotalJourney"
        component={L3_TotalJourneyStackNavigator}
      />
    </Drawer.Navigator>
  );
}

export default L2_AppDrawerNavigator;
