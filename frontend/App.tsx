/**
 * Sample React Native App
 * https://github.com/facebook/react-native
 *
 * @format
 */

import {NavigationContainer} from '@react-navigation/native';
import L1_RootStackNavigator from './src/navigations/L1_RootStackNavigator';
import {MD3LightTheme, Provider as PaperProvider} from 'react-native-paper';

const theme = {
  ...MD3LightTheme,
  colors: {
    ...MD3LightTheme.colors,
    primary: '#7173C9',
    secondary: '#DF94C2',
    tertiary: '#FFBDC1',
    quaternary: '#141060',
    surface: '#8398D1',
    accent: '#70558E',
    neutral: '#D9D9D9',
  },
};

function App(): JSX.Element {
  return (
    <PaperProvider theme={theme}>
      <NavigationContainer>
        <L1_RootStackNavigator />
      </NavigationContainer>
    </PaperProvider>
  );
}

export default App;
